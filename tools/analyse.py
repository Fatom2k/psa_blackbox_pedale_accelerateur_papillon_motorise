#!/usr/bin/env python3
"""
Analyse des enregistrements de la blackbox PSA (pédale + papillon motorisé).

  pip install pandas matplotlib

Exemples :
  # résumé des événements d'un démarrage
  python analyse.py events  bb0012_events.csv

  # tracé autour d'un événement (±5 s), journal en plusieurs parties possible
  python analyse.py journal bb0012_00.csv bb0012_01.csv --events bb0012_events.csv --t 754321 --fenetre 5

  # tout le journal (sous-échantillonné pour l'affichage)
  python analyse.py journal bb0012_00.csv --events bb0012_events.csv

  # capture rapide 2 kHz
  python analyse.py capture bb0012_cap_003.csv

Ajouter --png fichier.png pour enregistrer l'image au lieu de l'afficher.
"""
import argparse
import sys

import matplotlib
import pandas as pd

PEDAL_K = 2.0
FAST = ["V5P", "S1", "S2", "V5T", "T1", "T2"]


def lire_journal(fichiers):
    parts = [pd.read_csv(f, comment="#") for f in fichiers]
    df = pd.concat(parts, ignore_index=True).sort_values("t_ms").reset_index(drop=True)
    df["t"] = df["t_ms"] / 1000.0
    ref = (df["V5P"] - df["GNDP"]).where(lambda s: s > 3.0, 5.0)
    df["ecart_pedale_pct"] = ((df["S1"] - df["GNDP"]) - PEDAL_K * (df["S2"] - df["GNDP"])) / ref * 100
    df["T1+T2"] = df["T1"] + df["T2"] - 2 * df["GNDT"]
    df["12V_vu_calc"] = df["V12REL"] - df["GNDECU"]
    return df


def lire_events(fichier):
    if not fichier:
        return None
    ev = pd.read_csv(fichier)
    ev["t"] = ev["t_ms"] / 1000.0
    return ev


def cmd_events(a):
    ev = lire_events(a.fichier)
    if ev is None or ev.empty:
        print("Aucun événement.")
        return
    pd.set_option("display.width", 160)
    print(ev[["t_ms", "type", "phase", "voie", "valeur", "duree_ms", "capture", "fichier_journal"]].to_string(index=False))
    print("\nRésumé par type / voie (débuts et événements ponctuels) :")
    d = ev[ev["phase"] != "fin"]
    print(d.groupby(["type", "voie"]).size().to_string())
    fins = ev[ev["phase"] == "fin"]
    if not fins.empty:
        print("\nDurées (ms) :")
        print(fins.groupby(["type", "voie"])["duree_ms"].describe()[["count", "min", "max", "mean"]].to_string())


def marquer(ax, ev, t0, t1):
    if ev is None:
        return
    sel = ev[(ev["t"] >= t0) & (ev["t"] <= t1) & (ev["phase"] != "fin")]
    for _, e in sel.iterrows():
        couleur = "tab:cyan" if e["type"] == "MARQUEUR" else "tab:red"
        ax.axvline(e["t"], color=couleur, lw=1, alpha=0.7)


def bande(ax, df, voie, couleur, k=1.0):
    lo, hi = f"{voie}_min", f"{voie}_max"
    if lo in df and hi in df:
        ax.fill_between(df["t"], k * df[lo], k * df[hi], color=couleur, alpha=0.25, lw=0)


def cmd_journal(a):
    if a.png:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    df = lire_journal(a.fichiers)
    ev = lire_events(a.events)
    if a.t is not None:
        tc = a.t / 1000.0
        df = df[(df["t"] >= tc - a.fenetre) & (df["t"] <= tc + a.fenetre)]
    if df.empty:
        sys.exit("Aucune donnée dans cette fenêtre.")
    if len(df) > 200_000:                      # allège l'affichage d'un long trajet
        df = df.iloc[:: len(df) // 200_000 + 1]
    t0, t1 = df["t"].iloc[0], df["t"].iloc[-1]

    fig, ax = plt.subplots(6, 1, sharex=True, figsize=(14, 13))
    ax[0].set_title("Pédale (au calculateur) — bandes : min/max voie rapide 2 kHz")
    bande(ax[0], df, "S1", "tab:orange"); bande(ax[0], df, "S2", "tab:blue", PEDAL_K)
    ax[0].plot(df["t"], df["S1"], color="tab:orange", lw=0.8, label="S1")
    ax[0].plot(df["t"], PEDAL_K * df["S2"], color="tab:blue", lw=0.8, label="2 x S2")
    ax[0].set_ylabel("V"); ax[0].legend(loc="upper right")

    ax[1].plot(df["t"], df["ecart_pedale_pct"], color="tab:red", lw=0.8)
    ax[1].axhspan(-6, 6, color="tab:green", alpha=0.08)
    ax[1].set_ylabel("écart S1-2S2\n% du 5 V")

    bande(ax[2], df, "T1", "tab:purple"); bande(ax[2], df, "T2", "tab:green")
    ax[2].plot(df["t"], df["T1"], color="tab:purple", lw=0.8, label="piste 1")
    ax[2].plot(df["t"], df["T2"], color="tab:green", lw=0.8, label="piste 2")
    ax[2].plot(df["t"], df["T1+T2"], color="grey", lw=0.6, ls="--", label="T1+T2")
    ax[2].set_title("Papillon motorisé"); ax[2].set_ylabel("V"); ax[2].legend(loc="upper right")

    bande(ax[3], df, "V5P", "tab:orange"); bande(ax[3], df, "V5T", "tab:purple")
    ax[3].plot(df["t"], df["V5P"], color="tab:orange", lw=0.8, label="+5 V pédale")
    ax[3].plot(df["t"], df["V5T"], color="tab:purple", lw=0.8, label="+5 V papillon")
    ax[3].set_ylabel("V"); ax[3].legend(loc="upper right")

    ax[4].plot(df["t"], df["V12BAT"], lw=0.8, label="+BAT calculateur")
    ax[4].plot(df["t"], df["V12REL"], lw=0.8, label="+ après relais")
    ax[4].plot(df["t"], df["12V_vu_calc"], lw=0.8, ls="--", label="12 V vu par le calculateur")
    ax[4].set_ylabel("V"); ax[4].legend(loc="upper right")

    for c, lab in (("GNDP", "masse capteur pédale"), ("GNDT", "masse capteur papillon"), ("GNDECU", "masse calculateur")):
        ax[5].plot(df["t"], df[c] * 1000, lw=0.8, label=lab)
    ax[5].set_ylabel("mV / batterie -"); ax[5].legend(loc="upper right"); ax[5].set_xlabel("temps depuis la mise sous tension (s)")

    for x in ax:
        x.grid(alpha=0.3)
        marquer(x, ev, t0, t1)
    fig.tight_layout()
    if a.png:
        fig.savefig(a.png, dpi=110)
        print("Image :", a.png)
    else:
        plt.show()


def cmd_capture(a):
    if a.png:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(a.fichier, encoding="utf-8") as f:
        titre = f.readline().strip("# \n")
    df = pd.read_csv(a.fichier, comment="#")
    df["t"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    ttrig = None
    if "t_ms=" in titre:
        ttrig = (float(titre.split("t_ms=")[1].split()[0]) - df["t_ms"].iloc[0]) / 1000.0

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(14, 8))
    ax[0].plot(df["t"], df["S1"], lw=0.8, label="S1"); ax[0].plot(df["t"], PEDAL_K * df["S2"], lw=0.8, label="2 x S2")
    ax[0].set_title(titre); ax[0].legend(loc="upper right"); ax[0].set_ylabel("V")
    ax[1].plot(df["t"], df["T1"], lw=0.8, label="piste 1"); ax[1].plot(df["t"], df["T2"], lw=0.8, label="piste 2")
    ax[1].legend(loc="upper right"); ax[1].set_ylabel("V")
    ax[2].plot(df["t"], df["V5P"], lw=0.8, label="+5 V pédale"); ax[2].plot(df["t"], df["V5T"], lw=0.8, label="+5 V papillon")
    ax[2].legend(loc="upper right"); ax[2].set_ylabel("V"); ax[2].set_xlabel("s")
    for x in ax:
        x.grid(alpha=0.3)
        if ttrig is not None:
            x.axvline(ttrig, color="tab:red", lw=1)
    fig.tight_layout()
    if a.png:
        fig.savefig(a.png, dpi=110)
        print("Image :", a.png)
    else:
        plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = p.add_subparsers(dest="cmd", required=True)
    e = sp.add_parser("events", help="liste et résumé des événements")
    e.add_argument("fichier")
    e.set_defaults(func=cmd_events)
    j = sp.add_parser("journal", help="tracé du journal continu")
    j.add_argument("fichiers", nargs="+")
    j.add_argument("--events")
    j.add_argument("--t", type=float, help="centre de la fenêtre, en t_ms")
    j.add_argument("--fenetre", type=float, default=5.0, help="demi-largeur en secondes (défaut 5)")
    j.add_argument("--png")
    j.set_defaults(func=cmd_journal)
    c = sp.add_parser("capture", help="tracé d'une capture rapide 2 kHz")
    c.add_argument("fichier")
    c.add_argument("--png")
    c.set_defaults(func=cmd_capture)
    a = p.parse_args()
    a.func(a)


if __name__ == "__main__":
    main()
