import subprocess
import itertools
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import time

# Definiere den Suchraum für deine Parameter
a0_space = [0.5, 1.0, 1.5, 2.0] #social acceleration strength
a1_space = [0.05, 0.09, 0.15] #equilibrium speed
b0_space = [1.0, 2.0, 4.0, 7.0, 10.0] #social turning strength
b1_space = [0.05, 0.09, 0.15, 0.5, 1.0, 2.0] #equilibrium psi
gam_space = [1.0, 0.5, 0.1]


# Generiere alle möglichen Kombinationen
combinations = list(itertools.product(a0_space, a1_space, b0_space, b1_space, gam_space))

print(f"Starte Evaluierung von {len(combinations)} Parameter-Kombinationen...")

# Liste, um die finalen Ergebnisse jedes Durchlaufs zu speichern
all_results = []

for idx, (a0, a1, b0, b1, gam) in enumerate(combinations):
    print(f"\n--- Durchlauf {idx+1}/{len(combinations)} ---")
    print(f"Teste Parameter: a0={a0}, a1={a1}, b0={b0}, b1={b1}")
    
    # 1. MOOS Simulation starten
    process = subprocess.Popen([
        "./launch.sh", 
        str(a0), str(a1), str(b0), str(b1), str(gam)
    ])
    
    # 2. Simulation laufen lassen
    time.sleep(20) 
    
    # 3. Cleanup: Simulation beenden
    os.system("killall -q -9 pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger uTimerScript pFlockEvaluator")
    time.sleep(1)
    os.system("ktm")
    time.sleep(4) 
    os.system("killall -q -9 pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger uTimerScript pFlockEvaluator")
    time.sleep(1)
    os.system("ktm")    
    time.sleep(4) 

    # 4. AUSWERTUNG: Letzte Zeile der CSV auslesen
    print("\n--- Lese CSV-Datei ein ---")
    
    if not os.path.exists("flock_evaluation.csv"):
        print("KRITISCHER FEHLER: flock_evaluation.csv wurde nach dem Lauf nicht gefunden!")
        exit(1)
        
    file_size = os.path.getsize("flock_evaluation.csv")
    print(f"Datei gefunden. Größe: {file_size} Bytes")

    # Header is now longer due to the new metrics (~90 bytes)
    if file_size <= 95:
        print("WARNUNG: Simulation hat keine Daten erzeugt. Überspringe Durchlauf.")
        continue 
    
    # CSV Einlesen
    df_run = pd.read_csv("flock_evaluation.csv")
    print(f"Pandas hat {len(df_run)} Daten-Zeilen eingelesen.")
    
    if df_run.empty:
        print("WARNUNG: Die Datei existiert, besteht aber scheinbar nur aus der Kopfzeile!")
    else:
        last_row = df_run.iloc[-1]
        
        # Extract the new paper metrics
        pol_val = float(last_row["PolarizationOrder"])
        dist_val = float(last_row["MeanDistance"])
        clus_val = int(last_row["MaxClusterSize"])
        rca_val = float(last_row["AreaToCircleRatio"])
        overlap_val = float(last_row["OverlapRatio"])
        
        all_results.append({
            "a0": a0,
            "a1": a1,
            "b0": b0,
            "b1": b1,
            "gam": gam,
            "PolarizationOrder": pol_val,
            "MeanDistance": dist_val,
            "MaxClusterSize": clus_val,
            "AreaToCircleRatio": rca_val,
            "OverlapRatio": overlap_val
        })
        print(f"Erfolg! Werte gespeichert -> Pol: {pol_val:.2f}, Overlap: {overlap_val:.2f}")

print("\n=== Optimierung abgeschlossen ===")

df_final = pd.DataFrame(all_results)
df_final.to_csv("hyperparameter_results.csv", index=False)
print("Ergebnisse gespeichert in 'hyperparameter_results.csv'")

# --- VISUALISIERUNG ---
sns.set_theme(style="whitegrid")

# Plot 1: Trade-off zwischen Schwarm-Dichte (Distance) und Ausrichtung (Polarization)
plt.figure(figsize=(10, 6))
scatter = sns.scatterplot(
    data=df_final, 
    x="PolarizationOrder",  
    y="MeanDistance",       
    hue="OverlapRatio",     
    size="a0",            
    sizes=(50, 200),
    palette="flare"       
)

plt.title("Schwarm-Performance: Polarization vs. Mean Distance")
plt.xlabel("Polarization Order (1.0 = Perfect Alignment)")
plt.ylabel("Mean Inter-individual Distance [m]")
plt.xlim(0, 1.05)

plt.tight_layout()
plt.savefig("optimization_scatter.png", dpi=300)
plt.show()

# Plot 2: Korrelations-Matrix
plt.figure(figsize=(8, 6))
correlation_matrix = df_final.corr()
sns.heatmap(correlation_matrix, annot=True, cmap="coolwarm", fmt=".2f", vmin=-1, vmax=1)
plt.title("Korrelation zwischen Parametern und Metriken")
plt.tight_layout()
plt.savefig("optimization_correlation.png", dpi=300)
plt.show()