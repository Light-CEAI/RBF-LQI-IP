%%{init: {'theme': 'base', 'themeVariables': { 'primaryColor': '#E1F5FE', 'edgeLabelBackground':'#FFFFFF', 'tertiaryColor': '#FFFFFF'}}}%%
graph TD
    %% --- START BLOCK ---
    Start([<b>START:</b><br/>Inverted Pendulum<br/>RBF-LQI Control Project]) --> P1
    
    %% --- PHASE 1: SYSTEM IDENTIFICATION ---
    subgraph P1[<b>PHASE 1: Plant & Actuator Modeling</b>]
        direction TB
        M1(Physical System Identification) -->|Derive EOM via Physics| M2
        M2[<b>Non-Linear State-Space Model (True Plant)</b><br/>dot_x = f(x, u, D_tensor)]
        
        M3(Identify Baseline Constants: m, M, l, L)
        M4(Define Actuator Bounds & Non-Linearities: Sat(u), b_friction)
        M3 & M4 -->|System Parameters & Justification| M2
    end
    
    P1 --> P2
    
    %% --- PHASE 2: CONTROLLER DESIGN (SYNTHESIS) ---
    subgraph P2[<b>PHASE 2: Baseline LQI & RBF Design</b>]
        direction TB
        C1(Linearize Model about UP Position) -->|A_lin, B_lin| C2
        
        subgraph C2[<b>LQI Controller Synthesis</b>]
            L1(Define Penalty Matrices: Q, R) -->|design_lqi.m| L2
            L2[Solve CARE -> Calculate Gain Matrix K]
        end
        
        subgraph C3[<b>Adaptive RBF Network Design</b><br/>Focus: Lightweight, Light Precision]
            R1(Define Inputs [x, theta, ...])
            R2(Set Hidden Neurons n_hidden) -->|image_2.png| R1
            R3(Initialize centers C and widths sigma)
            R4[<b>Adaptive Update Law Selection</b><br/>Focus: Stability Loop Leakage/sigma-mod] -->|Prevent Parameter Drift| R3
        end
        
        C2 & C3 --> C4
        C4[<b>Integrated RBF-LQI Control Law</b><br/>u_net = u_lqi + u_ad(RBF)]
    end
    
    P2 -->|Embedded Target Code (C/C++)| P3
    
    %% --- PHASE 3: EMBEDDED SYSTEM CONSIDERATIONS ---
    subgraph P3[<b>PHASE 3: Embedded Deployment Considerations</b><br/>Key Focus: Resource Optimization (image_2.png)]
        direction TB
        E1(Quantization Assessment: float32, Fixed-Point)
        E2(Lookup Table LUT Approximation of Exp) -->|Optimize Lightweight RBF| E1
        E3(Computational Budget vs. Ts=0.01s)
        E4(Real-time State Estimation/Filter selection) -->|observer_design.m| E3
        E1 & E3 --> E5
        E5[<b>Embedded Loop Implementation Guardrails</b>]
    end
    
    P3 --> P4
    
    %% --- PHASE 4: STRESS TESTING & VALIDATION ---
    subgraph P4[<b>PHASE 4: Stress Testing & Data Analysis</b>]
        direction TB
        V1(Apply Nominal Boundaries Case [10deg initial])
        
        subgraph V2[<b>Unified Disturbance Environment (Section 2)</b>]
            D1(Define Sinusoidal Parametric Drift M_true[t])
            D2(Derive Unified Friction Tensor D_fric) -->|Shared State/Noise| D1
            D3[<b>Unified Disturbance Input tensor D_Tensor</b>]
        end
        
        V1 & V3 -->|Boundary Conditions & Forcing Functions| V4
        V4(<b>MONTE CARLO STRESS TEST</b><br/>2N Impulse + -20% Mass mismatch) -->|image_4.png| V5
        V5[Execute Comparative Simulation<br/>LQI Only vs. LQI+RBF]
    end
    
    P4 --> P5
    
    %% --- PHASE 5: VERIFICATION & COMPARISON ---
    subgraph P5[<b>PHASE 5: Performance Verification</b>]
        direction TB
        R1{<b>Check Stabilizing Performance</b><br/>(image_4.png Plots)}
        R2(Analyze theta, x, u_ad trajectory convergence) --> R1
        R3{<b>Check Embedded Limits</b>}
        R4(Check Execution < 10000us budget<br/>Validate Lightweight RBF feasibility) --> R3
        
        R1 & R3 --> R5
        R5{<b>Compare Baseline vs. Adaptive</b>}
    end
    
    P5 -->|Pass: RBF Compensates Drift & Noise| Fin
    P5 -->|Fail: Boundary Shutdown Violated| Fail
    
    %% --- LOOPBACK TO DESIGN ---
    Fail(FAIL: Stability Loss, Parameter Drift) -->|Retune RBF Rates/Gamma| P2
    Fail -->|Stiffen Centering Q_x| P2
    
    %% --- END BLOCK ---
    Fin([<b>FINISH:</b><br/>Deploy Verified<br/>Control to Target Rig])
    
    %% Styling
    classDef startend fill:#FFF9C4,stroke:#333,stroke-width:2px,color:#333;
    classDef subsystem fill:#E1F5FE,stroke:#333,stroke-width:1.5px,color:#333;
    classDef decision fill:#FFF3E0,stroke:#333,stroke-width:1.5px,color:#333;
    classDef highlight fill:#BBDEFB,stroke:#1B365D,stroke-width:2.5px,color:#1B365D;
    
    class Start,Fin startend;
    class M1,M2,M3,M4,L1,L2,R1,R2,R3,R4,C1,C4,E1,E2,E3,E4,E5,V1,V4,V5,D1,D2,D3,R2,R4 subsystem;
    class P1,P2,P3,P4,P5 highlight;
    class R1,R3,R5 decision;
