import subprocess
import itertools
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
import time

# Definiere den Suchraum für deine Parameter
a0_space = [0.1, 0.2, 0.3, 0.4]
a1_space = [1.5, 2.0, 2.5, 3.0]
b0_space = [0.1, 0.2, 0.3, 0.4]
b1_space = [0.1, 0.2, 0.3, 0.4]

# Generiere alle möglichen Kombinationen
combinations = list(itertools.product(a0_space, a1_space, b0_space, b1_space))

print(f"Starte Evaluierung von {len(combinations)} Parameter-Kombinationen...")

# Liste, um die finalen Ergebnisse jedes Durchlaufs zu speichern
all_results = []

#tick = 0

for idx, (a0, a1, b0, b1) in enumerate(combinations):
    #if tick >= 5:
        #break
    print(f"\n--- Durchlauf {idx+1}/{len(combinations)} ---")
    print(f"Teste Parameter: a0={a0}, a1={a1}, b0={b0}, b1={b1}")
    
    # 1. MOOS Simulation starten
    process = subprocess.Popen([
        "./launch.sh", 
        str(a0), str(a1), str(b0), str(b1)
    ])
    
    # 2. Simulation laufen lassen (z.B. 60 Sekunden)
    time.sleep(20) 
    
    # 3. Cleanup: Simulation beenden
    os.system("killall -q -9 pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger uTimerScript pFlockEvaluator")
    time.sleep(1)
    os.system("ktm")
    time.sleep(4) # Kurz warten, bis Dateien sicher geschlossen sind
    os.system("killall -q -9 pAntler MOOSDB pMarineViewer pShare uSimMarineV23 pHelmIvP pMarinePIDV22 pNodeReporter pSimVisionServer uProcessWatch pLogger uTimerScript pFlockEvaluator")
    time.sleep(1)
    os.system("ktm")    
    time.sleep(4) # Kurz warten, bis Dateien sicher geschlossen sind


    #tick += 1
    
# 4. AUSWERTUNG: Letzte Zeile der CSV auslesen (Jetzt mit voller Transparenz)
    print("\n--- Lese CSV-Datei ein ---")
    
    # Check 1: Ist die Datei überhaupt im richtigen Ordner?
    if not os.path.exists("flock_evaluation.csv"):
        print("KRITISCHER FEHLER: flock_evaluation.csv wurde nach dem Lauf nicht gefunden!")
        exit(1)
        
    # Check 2: Hat die Datei Inhalt?
    file_size = os.path.getsize("flock_evaluation.csv")
    print(f"Datei gefunden. Größe: {file_size} Bytes")

    # NEU: Wenn die Datei nur den Header hat (58 Bytes) oder ganz leer ist
    if file_size <= 58:
        print("WARNUNG: Simulation hat keine Daten erzeugt. Überspringe Durchlauf.")
        continue # Geht direkt zum nächsten Parameter-Set über
    
    # CSV Einlesen
    df_run = pd.read_csv("flock_evaluation.csv")
    print(f"Pandas hat {len(df_run)} Daten-Zeilen eingelesen.")
    
    if df_run.empty:
        print("WARNUNG: Die Datei existiert, besteht aber scheinbar nur aus der Kopfzeile!")
    else:
        # Check 3: Stimmen die Spaltennamen exakt überein?
        print("Gefundene Spalten:", df_run.columns.tolist())
        
        last_row = df_run.iloc[-1]
        
        # Werte extrahieren und anfügen (wir erzwingen hier Datentypen, 
        # falls Pandas etwas als String eingelesen hat)
        crashes_val = int(last_row["TotalCrashes"])
        heading_val = float(last_row["AvgHeadingVar"])
        area_val = float(last_row["AvgConvexArea"])
        
        all_results.append({
            "a0": a0,
            "a1": a1,
            "b0": b0,
            "b1": b1,
            "Crashes": crashes_val,
            "HeadingVar": heading_val,
            "ConvexArea": area_val
        })
        print(f"Erfolg! Werte gespeichert -> Crashes: {crashes_val}, Area: {area_val:.1f}")
            
    # Wir löschen die CSV NICHT mehr automatisch, bis wir den Fehler gefunden haben!
    # if os.path.exists("flock_evaluation.csv"):
    #     os.remove("flock_evaluation.csv")

print("\n=== Optimierung abgeschlossen ===")

# Konvertiere die gesammelten Daten in einen Pandas DataFrame
df_final = pd.DataFrame(all_results)

# Speichere das aggregierte Endergebnis als saubere CSV ab
df_final.to_csv("hyperparameter_results.csv", index=False)
print("Ergebnisse gespeichert in 'hyperparameter_results.csv'")

# --- VISUALISIERUNG ---
sns.set_theme(style="whitegrid")

# Plot 1: Trade-off zwischen Schwarm-Dichte (Area) und Ausrichtung (Heading Var)
plt.figure(figsize=(10, 6))
scatter = sns.scatterplot(
    data=df_final, 
    x="HeadingVar",       # x-Achse: Wie parallel fahren sie? (0 = perfekt)
    y="ConvexArea",       # y-Achse: Wie dicht sind sie? (kleiner = dichter)
    hue="Crashes",        # Farbe: Anzahl der Crashes
    size="a0",            # Größe der Punkte: Welchen Wert hatte a0?
    sizes=(50, 200),
    palette="flare"       # Farbschema (dunkel/rot = viele Crashes)
)

plt.title("Schwarm-Performance: Ausrichtung vs. Dichte")
plt.xlabel("Durchschnittliche Heading-Varianz (0 = Perfekt parallel)")
plt.ylabel("Durchschnittliche Konvexe Fläche [m²]")

# Invertiere die X-Achse leicht, falls 0 (besser) rechts stehen soll, 
# andernfalls ist links "besser".
plt.xlim(left=0) 

plt.tight_layout()
plt.savefig("optimization_scatter.png", dpi=300)
plt.show()

# Plot 2: Korrelations-Matrix (optional, aber extrem nützlich)
# Zeigt auf einen Blick, welcher Parameter (a0, a1...) welche Metrik am stärksten beeinflusst
plt.figure(figsize=(8, 6))
correlation_matrix = df_final.corr()
sns.heatmap(correlation_matrix, annot=True, cmap="coolwarm", fmt=".2f", vmin=-1, vmax=1)
plt.title("Korrelation zwischen Parametern und Metriken")
plt.tight_layout()
plt.savefig("optimization_correlation.png", dpi=300)
plt.show()