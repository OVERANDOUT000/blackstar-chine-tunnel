"""
Blackstar chine forebody campaign: reproduce every number and figure in README.md.

    python analysis/analyze.py

Reads   data/raw/blackstar_raw_2026-10-27.csv      (per-sample log exported by the rig)
        data/raw/blackstar_summary_2026-10-27.csv  (per-run metadata + calibration factor)
Writes  data/processed/runs.csv            one row per run, corrected lift
        data/processed/configurations.csv  one row per chine configuration (125)
        data/processed/no_chine.csv        no-chine reference per angle of attack
        analysis/results.json              every number quoted in the README
        figures/*.png

Processing in one paragraph
---------------------------
Each phase value is the mean of the last 2 s of that phase. The rig's original zero was a
straight line from the initial baseline (before the motors start) to the post baseline (after
they stop). But the post baseline starts while the rotors and tunnel air are still slowing, so
it still carries a residual load that is a fixed fraction k of the run's lift (k is fitted
across all runs). The corrected zero removes k * lift from the post baseline first, then
interpolates, so only the genuine run-specific zero shift is subtracted. Chine gain is
specimen lift minus the mean no-chine lift at the same angle of attack.
"""
import json
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm
import statsmodels.formula.api as smf
from statsmodels.stats.anova import anova_lm

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "data" / "raw"
PROC = ROOT / "data" / "processed"
FIG = ROOT / "figures"
PROC.mkdir(parents=True, exist_ok=True)
FIG.mkdir(parents=True, exist_ok=True)

WIN_MS = 2000                  # averaging window: last 2 s of each phase
REJECT = [37, 40, 41]          # early 0 deg runs contradicted by >1 g by their own retests
AOAS = [0, 5, 10, 15, 20]
LAMS = [15, 20, 25, 30, 35]
THS = [60, 62.5, 65, 67.5, 70]
PH_PRE, PH_M1SETTLE, PH_HOLD, PH_DOWN, PH_POST = 1, 3, 5, 6, 7

# ---------------------------------------------------------------- aerodynamic reference
# Dynamic pressure: water manometer, one tube facing the flow and one across it, read 6 mm at the
# test throttle. q = rho_water * g * h needs no air density. The +/-0.5 mm reading error is +/-8% in q.
G = 9.81
MANO_MM, MANO_ERR_MM = 6.0, 0.5
Q_PA = 1000.0 * G * MANO_MM / 1000.0
# Wing planform (docs/images/wing_planform_sketch.png), mm: 45 deg leading edges meeting a flat tip
# TIP_W wide, running out to the full SPAN at RAKE below the apex, then straight tips down to the
# trailing edge TAIL further back.
SPAN, TIP_W, RAKE, TAIL = 125.0, 14.375, 62.5, 23.4375
_h = TIP_W / 2
S_MM2 = (TIP_W + SPAN) / 2 * (RAKE - _h) + SPAN * TAIL            # planform area
_X = RAKE + TAIL
MAC_MM = 2 * ((_X - _h) ** 2 * _h + ((_X - _h) ** 3 - (_X - RAKE) ** 3) / 3) / S_MM2   # mean aerodynamic chord
ROOT_CHORD_MM = _X - _h
ASPECT = SPAN ** 2 / S_MM2
RHO_AIR, MU_AIR = 1.1, 1.85e-5                                    # Ranchi, about 650 m, 28 C (assumed)
V_MS = (2 * Q_PA / RHO_AIR) ** 0.5
RE_MAC = V_MS * MAC_MM / 1000 * RHO_AIR / MU_AIR
CL_PER_G = (G / 1000.0) / (Q_PA * S_MM2 * 1e-6)                    # CL = CL_PER_G * lift in grams

raw = pd.read_csv(RAW / "blackstar_raw_2026-10-27.csv")
meta = pd.read_csv(RAW / "blackstar_summary_2026-10-27.csv")
meta = meta[["run_id", "timestamp", "specimen_id", "aoa_deg", "lambda_pct", "theta_deg",
             "is_no_chine", "rep", "M1_pct", "M2_pct", "cal_counts_per_g"]]
meta["ts"] = pd.to_datetime(meta.timestamp)
factor = dict(zip(meta.run_id, meta.cal_counts_per_g))


def last_window(x, ph):
    y = x[x.phase == ph]
    return y[y.t_ms >= y.t_ms.max() - WIN_MS]


# ---------------------------------------------------------------- per-run reduction
rows = []
for rid, x in raw.groupby("run_id"):
    f = factor[rid]
    pre, post = last_window(x, PH_PRE), last_window(x, PH_POST)
    m1, hold = last_window(x, PH_M1SETTLE), last_window(x, PH_HOLD)
    z_pre, z_post = pre.raw_counts.mean(), post.raw_counts.mean()
    t_pre, t_post = pre.t_ms.mean(), post.t_ms.mean()
    p1 = x[x.phase == PH_PRE]
    rows.append(dict(
        run_id=rid,
        lift_pre_only=((z_pre - hold.raw_counts) / f).mean(),       # zero = initial baseline only
        m1_pre_only=((z_pre - m1.raw_counts) / f).mean(),
        post_offset=(z_pre - z_post) / f,                            # what the rig logged as "drift"
        w_hold=(hold.t_ms.mean() - t_pre) / (t_post - t_pre),        # interpolation weight at hold
        w_m1=(m1.t_ms.mean() - t_pre) / (t_post - t_pre),
        baseline_noise=pre.raw_counts.std() / abs(f),
        baseline_slope=np.polyfit(p1.t_ms / 1000, (z_pre - p1.raw_counts) / f, 1)[0],
        rate_hz=len(x) / ((x.t_ms.max() - x.t_ms.min()) / 1000),
    ))
runs = meta.merge(pd.DataFrame(rows), on="run_id")

# residual airflow fraction: post offset = k * lift + run-specific shift
k_tail, _ = np.polyfit(runs.lift_pre_only, runs.post_offset, 1)
r_tail = np.corrcoef(runs.lift_pre_only, runs.post_offset)[0, 1]
runs["zero_shift"] = runs.post_offset - k_tail * runs.lift_pre_only
runs["lift_g"] = runs.lift_pre_only - runs.w_hold * runs.zero_shift          # corrected lift
runs["m1_only_g"] = runs.m1_pre_only - runs.w_m1 * runs.zero_shift
runs["rejected"] = runs.run_id.isin(REJECT).astype(int)

ok = runs[runs.rejected == 0]
nc = ok[ok.is_no_chine == 1].groupby("aoa_deg").lift_g.agg(["count", "mean", "std"])
runs["chine_gain_g"] = np.where(runs.is_no_chine == 1, np.nan,
                                runs.lift_g - runs.aoa_deg.map(nc["mean"]))
runs["cl"] = runs.lift_g * CL_PER_G
runs["delta_cl"] = runs.chine_gain_g * CL_PER_G
ok = runs[runs.rejected == 0]
ch = ok[ok.is_no_chine == 0].copy()

# ---------------------------------------------------------------- per-configuration table
cfg = (ch.groupby(["aoa_deg", "lambda_pct", "theta_deg"])
         .agg(n_runs=("run_id", "size"), lift_g=("lift_g", "median"), chine_gain_g=("chine_gain_g", "median"))
         .reset_index())
cfg["gain_pct_of_no_chine"] = cfg.chine_gain_g / cfg.aoa_deg.map(nc["mean"]) * 100
cfg["cl"] = cfg.lift_g * CL_PER_G
cfg["delta_cl"] = cfg.chine_gain_g * CL_PER_G

# ---------------------------------------------------------------- statistics per angle
stats = {}
for a in AOAS:
    b = ch[ch.aoa_deg == a].copy()
    b["tmin"] = (b.ts - b.ts.min()).dt.total_seconds() / 60
    b = b.sort_values("ts").drop_duplicates(["lambda_pct", "theta_deg"], keep="last")   # one run per cell
    fit = smf.ols("chine_gain_g ~ C(theta_deg) + C(lambda_pct) + tmin", data=b).fit()
    an = anova_lm(fit, typ=2)
    c = cfg[cfg.aoa_deg == a]
    stats[a] = dict(mean_gain=c.chine_gain_g.mean(), sd_gain=c.chine_gain_g.std(),
                    pct=c.chine_gain_g.mean() / nc.loc[a, "mean"] * 100,
                    p_theta=an["PR(>F)"].iloc[0], p_lambda=an["PR(>F)"].iloc[1],
                    p_time=fit.pvalues["tmin"], slope_g_per_10min=fit.params["tmin"] * 10,
                    block_minutes=b.tmin.max())

# repeatability: rms deviation of repeated chined runs (5-20 deg)
dev = []
for _, g in ch[ch.aoa_deg > 0].groupby("specimen_id"):
    if len(g) > 1:
        dev += list(g.lift_g - g.lift_g.mean())
rep_rms = float(np.sqrt(np.mean(np.square(dev))))
# what the two simpler zeros would give, for the README's justification
dev_pre = []
for _, g in ch[ch.aoa_deg > 0].groupby("specimen_id"):
    if len(g) > 1:
        dev_pre += list(g.lift_pre_only - g.lift_pre_only.mean())
rep_rms_pre_only = float(np.sqrt(np.mean(np.square(dev_pre))))
dev_orig = []
for _, g in ch[ch.aoa_deg > 0].groupby("specimen_id"):
    if len(g) > 1:
        v = g.lift_pre_only - g.w_hold * g.post_offset
        dev_orig += list(v - v.mean())
rep_rms_original = float(np.sqrt(np.mean(np.square(dev_orig))))
ok_nc = ok[ok.is_no_chine == 1]
uncorr_nc20 = (ok_nc[ok_nc.aoa_deg == 20].lift_pre_only - ok_nc[ok_nc.aoa_deg == 20].w_hold * ok_nc[ok_nc.aoa_deg == 20].post_offset).mean()

# residual airflow tail after motor cut, % of hold lift
tail_rows = []
for rid, x in raw.groupby("run_id"):
    f = factor[rid]
    z_pre = last_window(x, PH_PRE).raw_counts.mean()
    hold = ((z_pre - last_window(x, PH_HOLD).raw_counts) / f).mean()
    t0 = x[x.phase == PH_POST].t_ms.min()
    p = x[x.phase.isin([PH_DOWN, PH_POST])]
    tail_rows.append(pd.DataFrame(dict(run_id=rid, tt=(p.t_ms - t0) / 1000,
                                       pct=(z_pre - p.raw_counts) / f / hold * 100)))
tail = pd.concat(tail_rows).merge(meta[["run_id", "aoa_deg"]], on="run_id")
tail = tail[tail.aoa_deg >= 5]
tail_halfsec = tail.assign(tb=(tail.tt * 2).round() / 2).groupby("tb").pct.mean()

# retest candidates: |z| >= 1.9 from an additive theta + lambda fit per angle
cand = []
for a in AOAS:
    b = ch[ch.aoa_deg == a].copy()
    fit = smf.ols("chine_gain_g ~ C(theta_deg) + C(lambda_pct)", data=b).fit()
    b["z"] = fit.resid / np.sqrt(fit.scale)
    cand.append(b)
cand = pd.concat(cand)
cand = cand[cand.z.abs() >= 1.9].sort_values("z", key=abs, ascending=False)

# ---------------------------------------------------------------- write data
cols = ["run_id", "timestamp", "specimen_id", "aoa_deg", "lambda_pct", "theta_deg", "is_no_chine", "rep",
        "M1_pct", "M2_pct", "cal_counts_per_g", "m1_only_g", "lift_g", "chine_gain_g", "cl", "delta_cl", "post_offset",
        "zero_shift", "baseline_noise", "rate_hz", "rejected"]
runs[cols].rename(columns={"post_offset": "post_baseline_offset_g", "zero_shift": "zero_shift_g",
                           "baseline_noise": "baseline_noise_sd_g"}).round(4).to_csv(PROC / "runs.csv", index=False)
cfg.round(4).to_csv(PROC / "configurations.csv", index=False)
nc_out = nc.rename(columns={"count": "n_runs", "mean": "lift_g", "std": "sd_g"})
nc_out["cl"] = nc_out.lift_g * CL_PER_G
nc_out.round(4).to_csv(PROC / "no_chine.csv")

c20 = cfg[cfg.aoa_deg == 20].sort_values("chine_gain_g", ascending=False)
R = dict(
    n_runs=int(len(runs)), n_chined_runs=int((runs.is_no_chine == 0).sum()), n_nc_runs=int((runs.is_no_chine == 1).sum()),
    n_configs=int(len(cfg)), rejected=REJECT, rate_hz=round(float(runs.rate_hz.median()), 1),
    k_tail_pct=round(k_tail * 100, 1), r_tail=round(r_tail, 2),
    tail_pct={str(k): round(float(v), 1) for k, v in tail_halfsec.items() if k >= 0},
    baseline_noise_g=round(float(runs.baseline_noise.mean()), 2),
    baseline_slope_max=round(float(runs.groupby("aoa_deg").baseline_slope.mean().abs().max()), 3),
    rep_rms_g=round(rep_rms, 3), rep_rms_pre_only_g=round(rep_rms_pre_only, 3),
    uncorrected_nc20=round(float(uncorr_nc20), 2),
    pre_only_nc20=round(float(ok_nc[ok_nc.aoa_deg == 20].lift_pre_only.mean()), 2),
    rep_rms_original_g=round(rep_rms_original, 3),
    no_chine={str(a): dict(n=int(r["count"]), lift=round(float(r["mean"]), 2),
                           sd=None if np.isnan(r["std"]) else round(float(r["std"]), 2)) for a, r in nc.iterrows()},
    stats={str(a): {k: round(float(v), 3) for k, v in s.items()} for a, s in stats.items()},
    lambda_effect={str(a): {str(int(l)): round(float(v), 2) for l, v in cfg[cfg.aoa_deg == a].groupby("lambda_pct").chine_gain_g.mean().items()} for a in AOAS},
    theta_effect={str(a): {f"{t:g}": round(float(v), 2) for t, v in cfg[cfg.aoa_deg == a].groupby("theta_deg").chine_gain_g.mean().items()} for a in AOAS},
    top5_20=[dict(lam=int(r.lambda_pct), th=f"{r.theta_deg:g}", gain=round(r.chine_gain_g, 2), pct=round(r.gain_pct_of_no_chine, 1), lift=round(r.lift_g, 2)) for r in c20.head(5).itertuples()],
    bottom5_20=[dict(lam=int(r.lambda_pct), th=f"{r.theta_deg:g}", gain=round(r.chine_gain_g, 2), pct=round(r.gain_pct_of_no_chine, 1), lift=round(r.lift_g, 2)) for r in c20.tail(5).itertuples()],
    pct20_range=[round(float(c20.gain_pct_of_no_chine.min()), 1), round(float(c20.gain_pct_of_no_chine.max()), 1)],
    wide_in_top10=int((c20.head(10).lambda_pct >= 30).sum()),
    retest=[dict(run=int(r.run_id), spec=r.specimen_id, z=round(r.z, 1)) for r in cand.itertuples()],
    calibrations=[dict(factor=round(float(f), 1), first=int(g.run_id.min()), last=int(g.run_id.max())) for f, g in meta.groupby("cal_counts_per_g")],
    blocks={str(a): dict(chined=int(((runs.aoa_deg == a) & (runs.is_no_chine == 0)).sum()), nc=int(((runs.aoa_deg == a) & (runs.is_no_chine == 1)).sum()),
                         minutes=int(round((runs[runs.aoa_deg == a].ts.max() - runs[runs.aoa_deg == a].ts.min()).total_seconds() / 60))) for a in AOAS},
    reference=dict(q_pa=round(Q_PA, 2), mano_mm=MANO_MM, mano_err_mm=MANO_ERR_MM, q_err_pct=round(MANO_ERR_MM / MANO_MM * 100, 1),
                   v_ms=round(V_MS, 2), rho_air=RHO_AIR, planform_mm2=round(S_MM2, 1), planform_cm2=round(S_MM2 / 100, 2),
                   span_mm=SPAN, root_chord_mm=ROOT_CHORD_MM, mac_mm=round(MAC_MM, 2), aspect_ratio=round(ASPECT, 2),
                   re_mac=int(round(RE_MAC, -2)), cl_per_g=round(CL_PER_G, 5)),
    cl_no_chine={str(a): round(float(r["mean"]) * CL_PER_G, 3) for a, r in nc.iterrows()},
    cl_chined_mean={str(a): round(float(cfg[cfg.aoa_deg == a].cl.mean()), 3) for a in AOAS},
    cl_best20=dict(lam=int(c20.iloc[0].lambda_pct), th=f"{c20.iloc[0].theta_deg:g}", cl=round(float(c20.iloc[0].cl), 3),
                   delta_cl=round(float(c20.iloc[0].delta_cl), 3)),
    delta_cl_mean={str(a): round(float(cfg[cfg.aoa_deg == a].delta_cl.mean()), 3) for a in AOAS},
)
(ROOT / "analysis" / "results.json").write_text(json.dumps(R, indent=1))

# ---------------------------------------------------------------- figures
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
BLUE, ORANGE = "#2a78d6", "#eb6834"
ORD = {0: "#86b6ef", 5: "#5598e7", 10: "#2a78d6", 15: "#1c5cab", 20: "#104281"}
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.facecolor": SURF, "figure.facecolor": SURF,
                     "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                     "axes.titlecolor": INK, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.grid": True,
                     "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
                     "legend.frameon": False, "legend.fontsize": 8, "savefig.dpi": 200})


def save(fig, name):
    fig.tight_layout()
    fig.savefig(FIG / name, bbox_inches="tight")
    plt.close(fig)


# 1. one run
rid = 157
x = raw[raw.run_id == rid]
f = factor[rid]
pre, post, hold = last_window(x, PH_PRE), last_window(x, PH_POST), last_window(x, PH_HOLD)
g = (pre.raw_counts.mean() - x.raw_counts) / f
names = {1: "baseline", 2: "M1 climb", 3: "M1 settle", 4: "M2 climb", 5: "hold", 7: "post\nbaseline"}
fig, ax = plt.subplots(figsize=(7.2, 3.3))
for i, (ph, y) in enumerate(x.groupby("phase")):
    a0, a1 = y.t_ms.min() / 1000, y.t_ms.max() / 1000
    if i % 2 == 0:
        ax.axvspan(a0, a1, color="#f0efec", lw=0, zorder=0)
    if ph in names:
        ax.text((a0 + a1) / 2, 24.6, names[ph], ha="center", va="top", fontsize=7, color=MUTED)
ax.plot(x.t_ms / 1000, g, color=BLUE, lw=1.5)
for w, lab in [(pre, "initial\nwindow"), (hold, "hold\nwindow"), (post, "post\nwindow")]:
    ax.plot([w.t_ms.min() / 1000, w.t_ms.max() / 1000], [-1.6, -1.6], color=INK2, lw=2.5, solid_capstyle="butt")
    ax.text(w.t_ms.mean() / 1000, -2.0, lab, ha="center", va="top", fontsize=7, color=INK2)
ax.annotate("lift still coming off:\nrotors and tunnel air\nstill slowing", xy=(post.t_ms.min() / 1000 - 0.3, 2.5),
            xytext=(16.6, 9), fontsize=7.5, color=INK, arrowprops=dict(arrowstyle="->", color=INK2, lw=0.8))
ax.set_ylim(-4.2, 25)
ax.set_xlabel("time since run start (s)")
ax.set_ylabel("lift (g)")
ax.set_title(f"One automated run: {meta.set_index('run_id').specimen_id[rid]}", loc="left")
save(fig, "fig1_run_trace.png")

# 2. airflow tail
fig, ax = plt.subplots(figsize=(7.2, 3.0))
lo = tail.tt.max() - 2.0
ax.axvspan(lo, tail.tt.max(), color="#f0efec", lw=0, zorder=0)
ax.text(lo + 1.0, 92, "post-baseline averaging window", ha="center", va="top", fontsize=7.5, color=INK2)
for a in [5, 10, 15, 20]:
    q = tail[tail.aoa_deg == a].assign(tb=lambda d: (d.tt / 0.1).round() * 0.1).groupby("tb").pct.mean()
    ax.plot(q.index, q.values, color=ORD[a], lw=1.6)
ax.axhline(0, color=AXIS, lw=0.8)
ax.axvline(0, color=INK2, lw=0.8, ls=":")
ax.text(0.03, 60, "post phase\nstarts", fontsize=7, color=INK2)
ax.text(0.55, 62, "5°, 10°, 15° and 20° lines lie on top of each other:\nthe tail is the same fraction of load at every angle",
        fontsize=7.5, color=INK)
ax.set_xlim(-0.4, 3.3)
ax.set_ylim(-3, 100)
ax.set_xlabel("time since post phase started (s)")
ax.set_ylabel("residual lift (% of hold)")
ax.set_title("After the motors cut, lift takes more than 3 s to die away", loc="left")
save(fig, "fig2_airflow_tail.png")

# 3. post offset scales with load
fig, ax = plt.subplots(figsize=(7.2, 3.0))
ax.scatter(runs.lift_pre_only, runs.post_offset, s=14, color=BLUE, alpha=0.75, edgecolor=SURF, lw=0.4)
xx = np.linspace(0, 24, 10)
ax.plot(xx, k_tail * xx + np.polyfit(runs.lift_pre_only, runs.post_offset, 1)[1], color=INK2, lw=1.2, ls="--")
ax.text(1, 2.75, f"offset ≈ {k_tail * 100:.1f}% of lift   (r = {r_tail:.2f}, {len(runs)} runs)", fontsize=8, color=INK)
ax.set_xlabel("hold lift relative to initial baseline (g)")
ax.set_ylabel("post-baseline offset (g)")
ax.set_title("The post-baseline offset scales with load: airflow, not sensor drift", loc="left")
save(fig, "fig3_offset_vs_load.png")

# 4. lift curve
cs = cfg.groupby("aoa_deg").lift_g.agg(["mean", "min", "max"])
fig, ax = plt.subplots(figsize=(7.2, 3.0))
ax.fill_between(cs.index, cs["min"], cs["max"], color=BLUE, alpha=0.18, lw=0)
ax.plot(cs.index, cs["mean"], color=BLUE, lw=1.8, marker="o", ms=4)
ax.plot(nc.index, nc["mean"], color=ORANGE, lw=1.8, marker="s", ms=4)
ax.text(20.5, cs["mean"].iloc[-1] + 0.5, "chined (mean,\nband = all 25)", color=INK2, fontsize=7.5, va="center")
ax.text(20.5, nc["mean"].iloc[-1] - 0.9, "no chine", color=INK2, fontsize=7.5, va="center")
ax.set_xticks(AOAS)
ax.set_xlim(-1, 24)
ax.set_xlabel("angle of attack (°)")
ax.set_ylabel("lift (g)")
ax.set_title("Lift curve: chined and plain noses only diverge above 10°", loc="left")
save(fig, "fig4_lift_curve.png")

# 5. chine gain vs angle, every configuration
fig, ax = plt.subplots(figsize=(7.2, 3.0))
rng = np.random.default_rng(1)
for a in AOAS:
    v = cfg[cfg.aoa_deg == a].chine_gain_g.values
    ax.scatter(a + rng.uniform(-0.7, 0.7, len(v)), v, s=12, color=BLUE, alpha=0.55, edgecolor=SURF, lw=0.3)
m = cfg.groupby("aoa_deg").chine_gain_g.mean()
ax.plot(m.index, m.values, color=INK, lw=1.6, marker="o", ms=4)
nc_sd = nc["std"].reindex(AOAS)
ax.fill_between(AOAS, -nc_sd.fillna(nc_sd.mean()) * 2, nc_sd.fillna(nc_sd.mean()) * 2, color=MUTED, alpha=0.15, lw=0)
ax.text(1.2, 0.42, "grey band: ±2 sd of no-chine repeats (noise floor)", fontsize=7, color=INK2)
ax.text(20.9, m.iloc[-1], "mean of 25", fontsize=7.5, color=INK2, va="center")
ax.axhline(0, color=AXIS, lw=0.8)
ax.set_xticks(AOAS)
ax.set_xlim(-1.5, 24)
ax.set_ylim(-0.6, 2.6)
ax.set_xlabel("angle of attack (°)")
ax.set_ylabel("chine lift gain (g)")
ax.set_title("Chine gain per configuration: nothing until 10°, then it grows", loc="left")
save(fig, "fig5_gain_vs_aoa.png")

# 6. heatmaps
cmap = LinearSegmentedColormap.from_list("div", ["#c43a39", "#f0efec", "#1c5cab"])
fig, axs = plt.subplots(1, 5, figsize=(7.4, 2.4), sharey=True)
for ax, a in zip(axs, AOAS):
    p = cfg[cfg.aoa_deg == a].pivot(index="lambda_pct", columns="theta_deg", values="chine_gain_g").reindex(index=LAMS, columns=THS)
    im = ax.imshow(p.values, cmap=cmap, norm=TwoSlopeNorm(0, -2.5, 2.5), aspect="equal")
    for i in range(5):
        for j in range(5):
            v = p.values[i, j]
            ax.text(j, i, f"{v:.1f}", ha="center", va="center", fontsize=5.8, color="white" if abs(v) > 1.4 else INK)
    ax.set_xticks(range(5))
    ax.set_xticklabels([f"{t:g}" for t in THS], fontsize=5.5, rotation=90)
    ax.set_yticks(range(5))
    ax.set_yticklabels(LAMS, fontsize=6.5)
    ax.grid(False)
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_title(f"α = {a}°", fontsize=8.5)
    ax.set_xlabel("θ (°)", fontsize=7)
axs[0].set_ylabel("λ (% of D)", fontsize=7.5)
cb = fig.colorbar(im, ax=axs, fraction=0.025, pad=0.015)
cb.ax.tick_params(labelsize=6)
cb.set_label("chine lift gain (g)", fontsize=7)
cb.outline.set_visible(False)
fig.savefig(FIG / "fig6_gain_heatmaps.png", bbox_inches="tight")
plt.close(fig)

# 7. main effects
fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 3.0), sharey=True)
for a in AOAS:
    q = cfg[cfg.aoa_deg == a]
    l, t = q.groupby("lambda_pct").chine_gain_g.mean(), q.groupby("theta_deg").chine_gain_g.mean()
    a1.plot(l.index, l.values, color=ORD[a], lw=1.6, marker="o", ms=3.5)
    a2.plot(t.index, t.values, color=ORD[a], lw=1.6, marker="o", ms=3.5)
    if a >= 15:
        a1.text(35.6, l.values[-1], f"{a}°", fontsize=7.5, color=INK2, va="center")
        a2.text(70.4, t.values[-1], f"{a}°", fontsize=7.5, color=INK2, va="center")
for ax in (a1, a2):
    ax.axhline(0, color=AXIS, lw=0.8)
a1.text(35.6, -0.05, "0°, 5°,\n10°", fontsize=7, color=INK2, va="center")
a2.text(70.4, -0.08, "0°, 5°,\n10°", fontsize=7, color=INK2, va="center")
a1.set_xticks(LAMS)
a1.set_xlim(14, 38)
a1.set_xlabel("chine width λ (% of D)")
a1.set_ylabel("chine lift gain (g)")
a2.set_xticks(THS)
a2.set_xticklabels([f"{t:g}" for t in THS])
a2.set_xlim(59, 72.8)
a2.set_xlabel("θ (°)")
a1.set_title("Wider chines lift more", loc="left")
a2.set_title("Lower θ lifts more", loc="left")
save(fig, "fig7_main_effects.png")

# 8. time-trend check
fig, axs = plt.subplots(1, 5, figsize=(7.4, 2.2), sharey=True)
for ax, a in zip(axs, AOAS):
    b = ch[ch.aoa_deg == a].copy()
    b["tmin"] = (b.ts - b.ts.min()).dt.total_seconds() / 60
    b = b.sort_values("ts").drop_duplicates(["lambda_pct", "theta_deg"], keep="last")
    ax.scatter(b.tmin, b.chine_gain_g, s=10, color=BLUE, edgecolor=SURF, lw=0.3)
    k1 = np.polyfit(b.tmin, b.chine_gain_g, 1)
    ax.plot([0, b.tmin.max()], np.polyval(k1, [0, b.tmin.max()]), color=INK2, lw=1, ls="--")
    ax.set_title(f"α = {a}°", fontsize=8.5)
    ax.set_xlabel("minutes into block", fontsize=7)
    ax.text(0.03, 0.04, f"p = {stats[a]['p_time']:.2f}", transform=ax.transAxes, fontsize=7, color=INK2)
    ax.tick_params(labelsize=6.5)
axs[0].set_ylabel("chine gain (g)", fontsize=7.5)
axs[0].set_ylim(-1.2, 3.0)
save(fig, "fig8_time_check.png")

# 9. lift coefficient
cm = cfg.groupby("aoa_deg").cl.agg(["mean", "min", "max"])
nc_cl = nc["mean"] * CL_PER_G
fig, ax = plt.subplots(figsize=(7.2, 3.0))
ax.fill_between(cm.index, cm["min"], cm["max"], color=BLUE, alpha=0.18, lw=0)
ax.plot(cm.index, cm["mean"], color=BLUE, lw=1.8, marker="o", ms=4)
ax.plot(nc_cl.index, nc_cl.values, color=ORANGE, lw=1.8, marker="s", ms=4)
qerr = MANO_ERR_MM / MANO_MM
ax.text(0.99, 0.05, f"every C$_L$ carries ±{qerr * 100:.0f}% from the manometer reading (q)", transform=ax.transAxes,
        ha="right", fontsize=7, color=INK2)
ax.text(20.5, cm["mean"].iloc[-1] + 0.02, "chined (mean,\nband = all 25)", color=INK2, fontsize=7.5, va="center")
ax.text(20.5, nc_cl.iloc[-1] - 0.04, "no chine", color=INK2, fontsize=7.5, va="center")
ax.set_xticks(AOAS)
ax.set_xlim(-1, 25)
ax.set_xlabel("angle of attack (°)")
ax.set_ylabel(f"lift coefficient $C_L$  (S = {S_MM2 / 100:.1f} cm²)")
ax.set_title("Lift coefficient on the wing planform area", loc="left")
save(fig, "fig9_lift_coefficient.png")

print(json.dumps({k: R[k] for k in ["n_runs", "k_tail_pct", "rep_rms_g", "rep_rms_pre_only_g", "pct20_range"]}))
print({a: (round(s["mean_gain"], 2), round(s["p_lambda"], 3), round(s["p_theta"], 3), round(s["p_time"], 3)) for a, s in stats.items()})
