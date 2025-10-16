# Dimamu_HVVA_Figures
publaction ready
# Dimamu_HVVA_Figures

Repository: **Dimamu_HVVA_Figures**

This canvas contains a GitHub-ready MATLAB package that generates Figures 4 → 14 for the HVVA manuscript. Files are provided below as code blocks; save each block as a separate `.m` file in a folder named `Dimamu_HVVA_Figures` and run `Generate_All_Figures.m` to create the PNGs.

---

## File: `Generate_All_Figures.m`

```matlab
% Generate_All_Figures.m
% Master script to run scripts for Figures 4->14
% Usage: place this file and all Fig*.m files in same folder and run.

function Generate_All_Figures()
    close all; clc;
    fprintf('Running Dimamu_HVVA_Figures package...\n');

    % Run each figure script (they are independent scripts/functions)
    Fig4_PowerComp();
    Fig5_TorqueComp();
    Fig6_FuelComp();
    Fig7_CombinedTorquePower();
    Fig8a_EngineOscillation();
    Fig8b_PressureSurface();
    Fig9_ValveLiftProfiles();
    Fig10_PendulumDisplacement();
    Fig11_DoublePendulum();
    Fig12_PressureDrop();
    Fig13_Validation_Torque();
    Fig14_Validation_Power();

    fprintf('All figures generated and saved as PNG files in: %s\n', pwd);
end
```

---

## File: `Fig4_PowerComp.m`

```matlab
% Fig4_PowerComp.m
% Figure 4: Power output comparison VVT vs VVA
function Fig4_PowerComp()
    engine.Pb_nominal = 6.6e3; % W
    power_incr_frac = 0.10; % +10%
    rpm = linspace(1000,7000,100);
    Pb_vvt = engine.Pb_nominal * (0.4 + 0.6 * ( (rpm - min(rpm))./(max(rpm)-min(rpm)) ).^1.5 );
    Pb_vva = Pb_vvt .* (1 + power_incr_frac .* (0.5 + 0.5*((rpm - min(rpm))./(max(rpm)-min(rpm)))));

    figure('Visible','off','Position',[100 100 900 420]);
    plot(rpm, Pb_vvt/1000,'-','LineWidth',1.6); hold on;
    plot(rpm, Pb_vva/1000,'--','LineWidth',1.6);
    xlabel('Engine speed (RPM)'); ylabel('Brake power (kW)');
    title('Figure 4 — Power output: VVT (baseline) vs VVA (HVVA)');
    legend('VVT (baseline)','VVA (HVVA)'); grid on;
    saveas(gcf,'Fig4_powerComp.png'); close(gcf);
end
```

---

## File: `Fig5_TorqueComp.m`

```matlab
% Fig5_TorqueComp.m
% Figure 5: Torque output comparison VVT vs VVA
function Fig5_TorqueComp()
    engine.Pb_nominal = 6.6e3;
    power_incr_frac = 0.10;
    rpm = linspace(1000,7000,100);
    Pb_vvt = engine.Pb_nominal * (0.4 + 0.6 * ( (rpm - min(rpm))./(max(rpm)-min(rpm)) ).^1.5 );
    Pb_vva = Pb_vvt .* (1 + power_incr_frac .* (0.5 + 0.5*((rpm - min(rpm))./(max(rpm)-min(rpm)))));
    Te_vvt = Pb_vvt ./ ((2*pi/60)*rpm);
    Te_vva = Pb_vva ./ ((2*pi/60)*rpm);

    figure('Visible','off','Position',[100 100 900 420]);
    plot(rpm, Te_vvt,'-','LineWidth',1.6); hold on;
    plot(rpm, Te_vva,'--','LineWidth',1.6);
    xlabel('Engine speed (RPM)'); ylabel('Torque (N·m)');
    title('Figure 5 — Torque output: VVT vs VVA');
    legend('VVT','VVA'); grid on;
    saveas(gcf,'Fig5_torqueComp.png'); close(gcf);
end
```

---

## File: `Fig6_FuelComp.m`

```matlab
% Fig6_FuelComp.m
% Figure 6: BSFC comparison baseline vs HVVA (uses manuscript reduction)
function Fig6_FuelComp()
    fuel_reduction_frac = 0.0564; % 5.64% reduction from manuscript
    rpm = linspace(1000,7000,100);
    BSFC_baseline = 350 - 80*((rpm - min(rpm))./(max(rpm)-min(rpm))).^0.9; % g/kWh
    BSFC_hvva = BSFC_baseline .* (1 - fuel_reduction_frac);

    figure('Visible','off','Position',[100 100 900 420]);
    plot(rpm, BSFC_baseline,'-','LineWidth',1.4); hold on;
    plot(rpm, BSFC_hvva,'--','LineWidth',1.4);
    xlabel('Engine speed (RPM)'); ylabel('BSFC (g/kWh)');
    title('Figure 6 — Fuel consumption (BSFC): Baseline vs HVVA');
    legend('Baseline','HVVA'); grid on;
    saveas(gcf,'Fig6_fuelComp.png'); close(gcf);
end
```

---

## File: `Fig7_CombinedTorquePower.m`

```matlab
% Fig7_CombinedTorquePower.m
% Figure 7: Combined torque and power overlay (VVT vs VVA)
function Fig7_CombinedTorquePower()
    engine.Pb_nominal = 6.6e3; power_incr_frac = 0.10; rpm = linspace(1000,7000,100);
    Pb_vvt = engine.Pb_nominal * (0.4 + 0.6 * ( (rpm - min(rpm))./(max(rpm)-min(rpm)) ).^1.5 );
    Pb_vva = Pb_vvt .* (1 + power_incr_frac .* (0.5 + 0.5*((rpm - min(rpm))./(max(rpm)-min(rpm)))));
    Te_vvt = Pb_vvt ./ ((2*pi/60)*rpm);
    Te_vva = Pb_vva ./ ((2*pi/60)*rpm);

    figure('Visible','off','Position',[100 100 900 420]);
    yyaxis left
    plot(rpm, Te_vvt,'-','LineWidth',1.5); hold on; plot(rpm, Te_vva,'--','LineWidth',1.5);
    ylabel('Torque (N·m)');
    yyaxis right
    plot(rpm, Pb_vvt/1000,':','LineWidth',1.6); plot(rpm, Pb_vva/1000,'-.','LineWidth',1.6);
    ylabel('Power (kW)'); xlabel('Engine speed (RPM)');
    title('Figure 7 — Combined torque & power: VVT vs VVA');
    legend({'Torque VVT','Torque VVA','Power VVT','Power VVA'},'Location','best'); grid on;
    saveas(gcf,'Fig7_combined.png'); close(gcf);
end
```

---

## File: `Fig8a_EngineOscillation.m`

```matlab
% Fig8a_EngineOscillation.m
% Figure 8a: Engine speed transient (representative)
function Fig8a_EngineOscillation()
    t = linspace(0,6,2000);
    engine_rpm = 1500 + 400*sin(2*pi*0.5*t) + 120*sin(2*pi*10*t).*exp(-0.3*t);
    figure('Visible','off','Position',[100 100 900 420]);
    plot(t, engine_rpm,'LineWidth',1.2); xlabel('Time (s)'); ylabel('Engine speed (RPM)');
    title('Figure 8a — Engine speed transient (constant throttle)'); grid on;
    saveas(gcf,'Fig8a_engineOscillation.png'); close(gcf);
end
```

---

## File: `Fig8b_PressureSurface.m`

```matlab
% Fig8b_PressureSurface.m
% Figure 8b: Hydraulic pressure vs valve lift vs duration (3D surface)
function Fig8b_PressureSurface()
    lift = linspace(0,22,80); duration = linspace(0,150,80);
    [L,D] = meshgrid(lift,duration);
    P_bar = 100 + 160*(1 - exp(-0.06*L)).*(1 - exp(-0.015*D));
    figure('Visible','off','Position',[100 100 900 700]); surf(L,D,P_bar,'EdgeColor','none');
    xlabel('Valve lift (mm)'); ylabel('Duration (°)'); zlabel('Pressure (bar)');
    title('Figure 8b — Pressure vs Lift vs Duration'); colorbar; view(45,30); grid on;
    saveas(gcf,'Fig8b_pressureSurface.png'); close(gcf);
end
```

---

## File: `Fig9_ValveLiftProfiles.m`

```matlab
% Fig9_ValveLiftProfiles.m
% Figure 9: Valve lift vs crank angle for multiple RPMs (VVT vs VVA)
function Fig9_ValveLiftProfiles()
    crank = linspace(0,720,721);
    rpms = [1000,3000,6000,7000];
    figure('Visible','off','Position',[100 100 900 700]);
    for i=1:length(rpms)
        N = rpms(i);
        lift_vvt = 10 + (6 * sin(2*pi*crank/720)).*(1 - 0.2*(N/8000));
        lift_vva = 10 + (8 * sin(2*pi*crank/720)).*(1 - 0.15*(N/8000));
        subplot(2,2,i);
        plot(crank, lift_vvt,'b-','LineWidth',1.2); hold on;
        plot(crank, lift_vva,'r--','LineWidth',1.2);
        xlabel('Crank angle (°)'); ylabel('Valve lift (mm)');
        title(sprintf('Valve lift @ %d RPM', N)); legend('VVT','VVA'); grid on;
    end
    saveas(gcf,'Fig9_valveLiftProfiles.png'); close(gcf);
end
```

---

## File: `Fig10_PendulumDisplacement.m`

```matlab
% Fig10_PendulumDisplacement.m
% Figure 10: Single-acting vs Double-acting pendulum displacement
function Fig10_PendulumDisplacement()
    tspan = linspace(0,60,2000);
    m = 0.08; k = 250; c = 0.2;
    [~, Xs] = ode45(@(t,x) [x(2); -(c/m)*x(2) - (k/m)*x(1)], tspan, [0.45; 0]);
    m1=0.06; m2=0.05; k1 = 0.9*k; k2 = 0.7*k; kc = 0.5*k; c1 = 0.15; c2=0.12; cc=0.08;
    [~, Yd] = ode45(@(t,y) twoDOF(t,y,m1,m2,k1,k2,kc,c1,c2,cc), tspan, [0.35;0;0.25;0]);
    figure('Visible','off','Position',[100 100 900 420]);
    plot(tspan, Xs(:,1),'r-','LineWidth',1.2); hold on; plot(tspan, Yd(:,1),'b--','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('Displacement (cm)'); title('Single vs Double pendulum displacement');
    legend('Single-acting','Double-acting'); grid on; xlim([0 60]);
    saveas(gcf,'Fig10_pendulumDisplacement.png'); close(gcf);
end

function dy = twoDOF(~,y,m1,m2,k1,k2,kc,c1,c2,cc)
    x1=y(1); v1=y(2); x2=y(3); v2=y(4);
    a1 = ( -k1*x1 - kc*(x1-x2) - c1*v1 - cc*(v1-v2) ) / m1;
    a2 = ( -k2*x2 - kc*(x2-x1) - c2*v2 - cc*(v2-v1) ) / m2;
    dy = [v1; a1; v2; a2];
end
```

---

## File: `Fig11_DoublePendulum.m`

```matlab
% Fig11_DoublePendulum.m
% Figure 11: Double pendulum angular displacement (damped nonlinear ODE)
function Fig11_DoublePendulum()
    L1 = 0.06; L2 = 0.06; m1 = 0.045; m2 = 0.035; b1 = 0.02; b2=0.015;
    tspan = linspace(0,10,2000);
    init = [0.5; 0; -0.7; 0];
    [t,y] = ode45(@(t,y) dpODE(t,y,m1,m2,L1,L2,b1,b2), tspan, init);
    figure('Visible','off','Position',[100 100 900 420]);
    plot(t,y(:,1),'LineWidth',1.2); hold on; plot(t,y(:,3),'--','LineWidth',1.2);
    xlabel('Time (s)'); ylabel('Angle (rad)'); title('Figure 11 — Double-pendulum'); legend('\theta_1','\theta_2'); grid on;
    saveas(gcf,'Fig11_doublePendulum.png'); close(gcf);
end

function dydt = dpODE(~,y,m1,m2,L1,L2,b1,b2)
    th1 = y(1); w1 = y(2); th2 = y(3); w2 = y(4);
    delta = th2 - th1;
    den1 = (m1 + m2)*L1 - m2*L1*cos(delta)^2;
    den2 = (L2/L1)*den1;
    a1 = ( m2*L1*w1^2*sin(delta)*cos(delta) + m2*9.81*sin(th2)*cos(delta) + m2*L2*w2^2*sin(delta) - (m1+m2)*9.81*sin(th1) ) / den1;
    a2 = ( -m2*L2*w2^2*sin(delta)*cos(delta) + (m1+m2)*( 9.81*sin(th1)*cos(delta) - L1*w1^2*sin(delta) - 9.81*sin(th2) ) ) / den2;
    a1 = a1 - b1*w1; a2 = a2 - b2*w2;
    dydt = [w1; a1; w2; a2];
end
```

---

## File: `Fig12_PressureDrop.m`

```matlab
% Fig12_PressureDrop.m
% Figure 12: Pressure drop vs RPM (exponential decay as in manuscript)
function Fig12_PressureDrop()
    rpm = linspace(500,7000,300);
    Pdrop = 4.5e-3 .* exp(-rpm/3000) + 0.05e-3*randn(size(rpm));
    Pdrop = max(Pdrop,0);
    figure('Visible','off','Position',[100 100 900 420]); semilogy(rpm,Pdrop,'LineWidth',1.4);
    xlabel('RPM'); ylabel('\DeltaP (Pa)'); title('Figure 12 — Pressure drop vs RPM'); grid on; saveas(gcf,'Fig12_pressureDrop.png'); close(gcf);
end
```

---

## File: `Fig13_Validation_Torque.m`

```matlab
% Fig13_Validation_Torque.m
% Figure 13: Validation torque using manuscript Table 3 values
function Fig13_Validation_Torque()
    rpm_table = [1992.972182, 98.243045, 1171.303, 409.9561, 765.7394];
    vva_current = [12.309904, 12.571885, 12.456869, 11.952077, 11.217252];
    vva_literature = [12.373802, 12.603834, 12.380192, 11.894569, 11.191693];
    rmse_val = sqrt(mean((vva_current - vva_literature).^2));

    figure('Visible','off','Position',[100 100 900 420]);
    plot(rpm_table, vva_literature,'o-','LineWidth',1.4); hold on;
    plot(rpm_table, vva_current,'s--','LineWidth',1.4);
    xlabel('Engine speed (RPM)'); ylabel('Torque (N·m)'); title(sprintf('Figure 13 — Validation torque (RMSE=%.4f)', rmse_val));
    legend('VVA Literature','VVA Current'); grid on; saveas(gcf,'Fig13_validation_torque.png'); close(gcf);
end
```

---

## File: `Fig14_Validation_Power.m`

```matlab
% Fig14_Validation_Power.m
% Figure 14: Validation power using manuscript Table 4 values
function Fig14_Validation_Power()
    rpm = [1000, 1019.08, 200, 320, 404, 599, 699];
    lit_power = [10, 12.97, 1.59, 3.5, 2.93, 8.41, 0.46];
    cur_power = [12.48, 16.44, 25.6, 49.63, 68.95, 86.28, 75.39];
    rmse_val = sqrt(mean((cur_power - lit_power).^2));
    figure('Visible','off','Position',[100 100 900 420]);
    plot(rpm, lit_power,'d-','LineWidth',1.4); hold on; plot(rpm, cur_power,'p--','LineWidth',1.4);
    xlabel('Engine speed (RPM)'); ylabel('Power (PS)'); title(sprintf('Figure 14 — Validation power (RMSE=%.3f)', rmse_val));
    legend('Literature','Current Work'); grid on; saveas(gcf,'Fig14_validation_power.png'); close(gcf);
end
```

---

## File: `README.md`

```markdown
# Dimamu_HVVA_Figures

This repository reproduces Figures 4 to 14 from the HVVA manuscript (Dimamu Biru et al.).

## How to run
1. Place all `.m` files in the same folder.  
2. In MATLAB, set the working directory to the folder.  
3. Run: `Generate_All_Figures`  
4. PNG files `Fig4_powerComp.png` ... `Fig14_validation_power.png` will appear in the folder.

## Key parameters (from manuscript)
- HVVA max pressure: 20 MPa
- Actuator stroke: 8 mm
- Response time: 10 ms
- Valve spring rate k: 250 N/m
- Valve spring preload: 20 N
- Base engine brake power (Lifan 177F): 6.6 kW
- Reported improvements used: +10% power, -5.64% fuel

## Equations used
- Brake power: $P_b = T_e \cdot \omega$ where $\omega = 2\pi N/60$.  
- Torque: $T_e = P_b / \omega$.  
- BSFC: $\text{BSFC (g/kWh)} = \dfrac{\dot{m}_{fuel}\cdot 3600}{P_b} \times 1000$.  
- Hydraulic force: $F = P \cdot A$.  
- Valve spring force: $F_s = k x + F_{preload}$.  
- Double pendulum ODEs: implemented in `Fig11_DoublePendulum.m`.

## Notes
- The code uses the numerical values and table snippets included in the manuscript. Replace arrays marked in code with full experimental tabular data if you require exact point-for-point reproduction.

## License
MIT — see LICENSE file for details.
```

---

## File: `LICENSE`

```text
MIT License

Copyright (c) 2025 Dimamu Biru

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

