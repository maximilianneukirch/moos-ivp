import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import plotly.express as px

print("Lese hyperparameter_results.csv ein...")
df = pd.read_csv('hyperparameter_results.csv')

# --- PLOT 1: Die Korrelationsmatrix ---
# Zeigt globale, lineare Zusammenhänge zwischen allen Variablen
plt.figure(figsize=(10, 8))
# Wir berechnen die Spearman-Korrelation (besser für diskrete Hyperparameter)
corr = df.corr(method='spearman')
sns.heatmap(corr, annot=True, cmap='coolwarm', fmt=".2f", center=0, vmin=-1, vmax=1)
plt.title("Korrelationsmatrix: Welcher Parameter steuert was?")
plt.tight_layout()
plt.savefig("01_korrelation.png", dpi=300)
plt.close()
print("-> 01_korrelation.png erstellt.")

# --- PLOT 2: Marginal Effects (Boxplots) ---
# Zeigt isoliert, wie das Verstellen eines Parameters die Metriken verschiebt
params = ['a0', 'a1', 'b0', 'b1', 'gam']
metrics = ['Crashes', 'HeadingVar', 'ConvexArea']

# Erstelle ein Raster von Plots (3 Zeilen für Metriken, 5 Spalten für Parameter)
fig, axes = plt.subplots(len(metrics), len(params), figsize=(20, 10))

for i, metric in enumerate(metrics):
    for j, param in enumerate(params):
        # Zeichne einen Boxplot für jede Parameter-Stufe
        sns.boxplot(data=df, x=param, y=metric, ax=axes[i, j], palette="Blues", hue=param, legend=False)
        
        # Achsenbeschriftungen nur außen für bessere Lesbarkeit
        if i == 0: axes[i, j].set_title(f"Einfluss von {param}")
        if j > 0: axes[i, j].set_ylabel("")
        if i < len(metrics) - 1: axes[i, j].set_xlabel("")

plt.tight_layout()
plt.savefig("02_parameter_einfluss.png", dpi=300)
plt.close()
print("-> 02_parameter_einfluss.png erstellt.")

# --- PLOT 3: Interaktiver Parallel Coordinates Plot ---
# Hiermit lassen sich "Pfade" von guten Parameter-Kombinationen visualisieren
# Wir sortieren die Daten, damit die besten (wenig Crashes) eine gute Farbe bekommen
fig = px.parallel_coordinates(
    df,
    color="Crashes",
    dimensions=['a0', 'a1', 'b0', 'b1', 'gam', 'ConvexArea', 'HeadingVar', 'Crashes'],
    color_continuous_scale=px.colors.diverging.RdYlBu_r, # Blau = Wenig Crashes, Rot = Viele
    title="Hyperparameter Trade-off Analyse (Interaktiv)"
)

# Speichere den Plot als interaktive HTML-Datei
fig.write_html("03_hyperparameter_dashboard.html")
print("-> 03_hyperparameter_dashboard.html erstellt.")
print("\nFühre das Skript aus und öffne die HTML-Datei in deinem Webbrowser!")