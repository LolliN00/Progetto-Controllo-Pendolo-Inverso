# Progetto Pendolo Inverso - Control Engineering Laboratory

Un sistema di controllo real-time per pendolo inverso implementato su Raspberry Pi con interfaccia MATLAB/Simulink.

## Autori
- **Giammarco Tonti** (Dottorando) - Sviluppo hardware, framework e interfacce
- **Lorenzo Nobili** - Implementazione del controllore

## Descrizione del Progetto

Questo progetto implementa un sistema di controllo per stabilizzare un pendolo inverso utilizzando un approccio di pole placement.  Il sistema è stato sviluppato nel contesto del corso di Control Engineering Laboratory presso l'Università degli Studi di Parma.

### Software
- **pendulum_control.c**: Codice principale del controllo real-time
- **gui_interface.mlapp**: Interfaccia grafica MATLAB
- **pendulum_design.m**: Progettazione del controllore pole placement
- **control_analysis.m**: Analisi di stabilità del sistema
- **pendulum_analysis.ipynb**: Notebook per modellazione matematica pendolo

## Parametri di Controllo

Il controllore implementato utilizza i seguenti guadagni ottimizzati:
- K1 = -0.3043
- K2 = 9.2119
- K3 = -1.4322
- K4 = 1.7199

Legge di controllo: `v_in = -K1*θ1 - K2*θ2 - K3*θ1_dot - K4*θ2_dot`

Dove:
- θ1: angolo motore
- θ2: angolo pendolo
- θ1_dot, θ2_dot: velocità angolari (filtrate)

