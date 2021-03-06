(define (problem schedule-47-3)
(:domain schedule)
(:objects
    A2
    Z1
    W1
    V1
    U1
    S1
    R1
    P1
    Q1
    O1
    N1
    M1
    L1
    K1
    J1
    I1
    H1
    G1
    F1
    E1
    D1
    C1
    B1
    A1
    Z0
    W0
    V0
    U0
    S0
    R0
    P0
    Q0
    O0
    N0
    M0
    L0
    K0
    J0
    I0
    H0
    G0
    F0
    E0
    D0
    C0
    CIRCULAR
    TWO
    THREE
    BLUE
    YELLOW
    BACK
    RED
    B0
    FRONT
    ONE
    BLACK
    OBLONG
    A0
)
(:init
    (idle punch) (idle drill-press) (idle lathe) (idle roller) (idle polisher)
    (idle immersion-painter) (idle spray-painter) (idle grinder) (ru unwantedargs)
    (SHAPE A0 OBLONG)
    (SURFACE-CONDITION A0 ROUGH)
    (PAINTED A0 BLACK)
    (HAS-HOLE A0 TWO BACK) (lasthole A0 TWO BACK) (linked A0 nowidth noorient TWO BACK)
    (TEMPERATURE A0 COLD)
    (SHAPE B0 CIRCULAR)
    (SURFACE-CONDITION B0 ROUGH)
    (PAINTED B0 BLUE)
    (HAS-HOLE B0 TWO BACK) (lasthole B0 TWO BACK) (linked B0 nowidth noorient TWO BACK)
    (TEMPERATURE B0 COLD)
    (SHAPE C0 CYLINDRICAL)
    (SURFACE-CONDITION C0 SMOOTH)
    (PAINTED C0 RED)
    (HAS-HOLE C0 ONE FRONT) (lasthole C0 ONE FRONT) (linked C0 nowidth noorient ONE FRONT)
    (TEMPERATURE C0 COLD)
    (SHAPE D0 OBLONG)
    (SURFACE-CONDITION D0 SMOOTH)
    (PAINTED D0 BLACK)
    (HAS-HOLE D0 ONE FRONT) (lasthole D0 ONE FRONT) (linked D0 nowidth noorient ONE FRONT)
    (TEMPERATURE D0 COLD)
    (SHAPE E0 CIRCULAR)
    (SURFACE-CONDITION E0 POLISHED)
    (PAINTED E0 BLACK)
    (HAS-HOLE E0 ONE BACK) (lasthole E0 ONE BACK) (linked E0 nowidth noorient ONE BACK)
    (TEMPERATURE E0 COLD)
    (SHAPE F0 CYLINDRICAL)
    (SURFACE-CONDITION F0 SMOOTH)
    (PAINTED F0 BLUE)
    (HAS-HOLE F0 ONE BACK) (lasthole F0 ONE BACK) (linked F0 nowidth noorient ONE BACK)
    (TEMPERATURE F0 COLD)
    (SHAPE G0 CIRCULAR)
    (SURFACE-CONDITION G0 ROUGH)
    (PAINTED G0 BLACK)
    (HAS-HOLE G0 TWO BACK) (lasthole G0 TWO BACK) (linked G0 nowidth noorient TWO BACK)
    (TEMPERATURE G0 COLD)
    (SHAPE H0 CYLINDRICAL)
    (SURFACE-CONDITION H0 SMOOTH)
    (PAINTED H0 BLUE)
    (HAS-HOLE H0 THREE FRONT) (lasthole H0 THREE FRONT) (linked H0 nowidth noorient THREE FRONT)
    (TEMPERATURE H0 COLD)
    (SHAPE I0 CYLINDRICAL)
    (SURFACE-CONDITION I0 SMOOTH)
    (PAINTED I0 RED)
    (HAS-HOLE I0 THREE BACK) (lasthole I0 THREE BACK) (linked I0 nowidth noorient THREE BACK)
    (TEMPERATURE I0 COLD)
    (SHAPE J0 CYLINDRICAL)
    (SURFACE-CONDITION J0 ROUGH)
    (PAINTED J0 BLUE)
    (HAS-HOLE J0 THREE FRONT) (lasthole J0 THREE FRONT) (linked J0 nowidth noorient THREE FRONT)
    (TEMPERATURE J0 COLD)
    (SHAPE K0 CIRCULAR)
    (SURFACE-CONDITION K0 ROUGH)
    (PAINTED K0 YELLOW)
    (HAS-HOLE K0 ONE FRONT) (lasthole K0 ONE FRONT) (linked K0 nowidth noorient ONE FRONT)
    (TEMPERATURE K0 COLD)
    (SHAPE L0 OBLONG)
    (SURFACE-CONDITION L0 SMOOTH)
    (PAINTED L0 RED)
    (HAS-HOLE L0 THREE BACK) (lasthole L0 THREE BACK) (linked L0 nowidth noorient THREE BACK)
    (TEMPERATURE L0 COLD)
    (SHAPE M0 OBLONG)
    (SURFACE-CONDITION M0 SMOOTH)
    (PAINTED M0 RED)
    (HAS-HOLE M0 THREE FRONT) (lasthole M0 THREE FRONT) (linked M0 nowidth noorient THREE FRONT)
    (TEMPERATURE M0 COLD)
    (SHAPE N0 CIRCULAR)
    (SURFACE-CONDITION N0 SMOOTH)
    (PAINTED N0 BLUE)
    (HAS-HOLE N0 TWO FRONT) (lasthole N0 TWO FRONT) (linked N0 nowidth noorient TWO FRONT)
    (TEMPERATURE N0 COLD)
    (SHAPE O0 OBLONG)
    (SURFACE-CONDITION O0 POLISHED)
    (PAINTED O0 RED)
    (HAS-HOLE O0 THREE FRONT) (lasthole O0 THREE FRONT) (linked O0 nowidth noorient THREE FRONT)
    (TEMPERATURE O0 COLD)
    (SHAPE Q0 CYLINDRICAL)
    (SURFACE-CONDITION Q0 ROUGH)
    (PAINTED Q0 RED)
    (HAS-HOLE Q0 THREE BACK) (lasthole Q0 THREE BACK) (linked Q0 nowidth noorient THREE BACK)
    (TEMPERATURE Q0 COLD)
    (SHAPE P0 OBLONG)
    (SURFACE-CONDITION P0 ROUGH)
    (PAINTED P0 BLACK)
    (HAS-HOLE P0 ONE FRONT) (lasthole P0 ONE FRONT) (linked P0 nowidth noorient ONE FRONT)
    (TEMPERATURE P0 COLD)
    (SHAPE R0 CIRCULAR)
    (SURFACE-CONDITION R0 ROUGH)
    (PAINTED R0 YELLOW)
    (HAS-HOLE R0 ONE BACK) (lasthole R0 ONE BACK) (linked R0 nowidth noorient ONE BACK)
    (TEMPERATURE R0 COLD)
    (SHAPE S0 OBLONG)
    (SURFACE-CONDITION S0 ROUGH)
    (PAINTED S0 YELLOW)
    (HAS-HOLE S0 TWO FRONT) (lasthole S0 TWO FRONT) (linked S0 nowidth noorient TWO FRONT)
    (TEMPERATURE S0 COLD)
    (SHAPE U0 CYLINDRICAL)
    (SURFACE-CONDITION U0 POLISHED)
    (PAINTED U0 BLACK)
    (HAS-HOLE U0 THREE BACK) (lasthole U0 THREE BACK) (linked U0 nowidth noorient THREE BACK)
    (TEMPERATURE U0 COLD)
    (SHAPE V0 OBLONG)
    (SURFACE-CONDITION V0 ROUGH)
    (PAINTED V0 RED)
    (HAS-HOLE V0 TWO FRONT) (lasthole V0 TWO FRONT) (linked V0 nowidth noorient TWO FRONT)
    (TEMPERATURE V0 COLD)
    (SHAPE W0 CYLINDRICAL)
    (SURFACE-CONDITION W0 ROUGH)
    (PAINTED W0 RED)
    (HAS-HOLE W0 THREE FRONT) (lasthole W0 THREE FRONT) (linked W0 nowidth noorient THREE FRONT)
    (TEMPERATURE W0 COLD)
    (SHAPE Z0 OBLONG)
    (SURFACE-CONDITION Z0 SMOOTH)
    (PAINTED Z0 RED)
    (HAS-HOLE Z0 THREE FRONT) (lasthole Z0 THREE FRONT) (linked Z0 nowidth noorient THREE FRONT)
    (TEMPERATURE Z0 COLD)
    (SHAPE A1 CYLINDRICAL)
    (SURFACE-CONDITION A1 ROUGH)
    (PAINTED A1 BLACK)
    (HAS-HOLE A1 ONE BACK) (lasthole A1 ONE BACK) (linked A1 nowidth noorient ONE BACK)
    (TEMPERATURE A1 COLD)
    (SHAPE B1 CIRCULAR)
    (SURFACE-CONDITION B1 SMOOTH)
    (PAINTED B1 YELLOW)
    (HAS-HOLE B1 THREE BACK) (lasthole B1 THREE BACK) (linked B1 nowidth noorient THREE BACK)
    (TEMPERATURE B1 COLD)
    (SHAPE C1 CIRCULAR)
    (SURFACE-CONDITION C1 POLISHED)
    (PAINTED C1 RED)
    (HAS-HOLE C1 TWO BACK) (lasthole C1 TWO BACK) (linked C1 nowidth noorient TWO BACK)
    (TEMPERATURE C1 COLD)
    (SHAPE D1 CYLINDRICAL)
    (SURFACE-CONDITION D1 POLISHED)
    (PAINTED D1 RED)
    (HAS-HOLE D1 TWO BACK) (lasthole D1 TWO BACK) (linked D1 nowidth noorient TWO BACK)
    (TEMPERATURE D1 COLD)
    (SHAPE E1 CIRCULAR)
    (SURFACE-CONDITION E1 SMOOTH)
    (PAINTED E1 BLUE)
    (HAS-HOLE E1 TWO FRONT) (lasthole E1 TWO FRONT) (linked E1 nowidth noorient TWO FRONT)
    (TEMPERATURE E1 COLD)
    (SHAPE F1 CIRCULAR)
    (SURFACE-CONDITION F1 POLISHED)
    (PAINTED F1 BLACK)
    (HAS-HOLE F1 THREE BACK) (lasthole F1 THREE BACK) (linked F1 nowidth noorient THREE BACK)
    (TEMPERATURE F1 COLD)
    (SHAPE G1 CIRCULAR)
    (SURFACE-CONDITION G1 SMOOTH)
    (PAINTED G1 YELLOW)
    (HAS-HOLE G1 THREE BACK) (lasthole G1 THREE BACK) (linked G1 nowidth noorient THREE BACK)
    (TEMPERATURE G1 COLD)
    (SHAPE H1 CYLINDRICAL)
    (SURFACE-CONDITION H1 SMOOTH)
    (PAINTED H1 YELLOW)
    (HAS-HOLE H1 THREE FRONT) (lasthole H1 THREE FRONT) (linked H1 nowidth noorient THREE FRONT)
    (TEMPERATURE H1 COLD)
    (SHAPE I1 CIRCULAR)
    (SURFACE-CONDITION I1 SMOOTH)
    (PAINTED I1 BLACK)
    (HAS-HOLE I1 ONE FRONT) (lasthole I1 ONE FRONT) (linked I1 nowidth noorient ONE FRONT)
    (TEMPERATURE I1 COLD)
    (SHAPE J1 OBLONG)
    (SURFACE-CONDITION J1 ROUGH)
    (PAINTED J1 YELLOW)
    (HAS-HOLE J1 THREE FRONT) (lasthole J1 THREE FRONT) (linked J1 nowidth noorient THREE FRONT)
    (TEMPERATURE J1 COLD)
    (SHAPE K1 CIRCULAR)
    (SURFACE-CONDITION K1 POLISHED)
    (PAINTED K1 BLUE)
    (HAS-HOLE K1 THREE FRONT) (lasthole K1 THREE FRONT) (linked K1 nowidth noorient THREE FRONT)
    (TEMPERATURE K1 COLD)
    (SHAPE L1 OBLONG)
    (SURFACE-CONDITION L1 SMOOTH)
    (PAINTED L1 RED)
    (HAS-HOLE L1 TWO FRONT) (lasthole L1 TWO FRONT) (linked L1 nowidth noorient TWO FRONT)
    (TEMPERATURE L1 COLD)
    (SHAPE M1 CIRCULAR)
    (SURFACE-CONDITION M1 POLISHED)
    (PAINTED M1 BLACK)
    (HAS-HOLE M1 ONE FRONT) (lasthole M1 ONE FRONT) (linked M1 nowidth noorient ONE FRONT)
    (TEMPERATURE M1 COLD)
    (SHAPE N1 CIRCULAR)
    (SURFACE-CONDITION N1 ROUGH)
    (PAINTED N1 RED)
    (HAS-HOLE N1 THREE FRONT) (lasthole N1 THREE FRONT) (linked N1 nowidth noorient THREE FRONT)
    (TEMPERATURE N1 COLD)
    (SHAPE O1 OBLONG)
    (SURFACE-CONDITION O1 SMOOTH)
    (PAINTED O1 BLUE)
    (HAS-HOLE O1 ONE FRONT) (lasthole O1 ONE FRONT) (linked O1 nowidth noorient ONE FRONT)
    (TEMPERATURE O1 COLD)
    (SHAPE Q1 CIRCULAR)
    (SURFACE-CONDITION Q1 SMOOTH)
    (PAINTED Q1 RED)
    (HAS-HOLE Q1 TWO BACK) (lasthole Q1 TWO BACK) (linked Q1 nowidth noorient TWO BACK)
    (TEMPERATURE Q1 COLD)
    (SHAPE P1 CYLINDRICAL)
    (SURFACE-CONDITION P1 SMOOTH)
    (PAINTED P1 BLACK)
    (HAS-HOLE P1 TWO BACK) (lasthole P1 TWO BACK) (linked P1 nowidth noorient TWO BACK)
    (TEMPERATURE P1 COLD)
    (SHAPE R1 CYLINDRICAL)
    (SURFACE-CONDITION R1 ROUGH)
    (PAINTED R1 RED)
    (HAS-HOLE R1 TWO BACK) (lasthole R1 TWO BACK) (linked R1 nowidth noorient TWO BACK)
    (TEMPERATURE R1 COLD)
    (SHAPE S1 OBLONG)
    (SURFACE-CONDITION S1 POLISHED)
    (PAINTED S1 RED)
    (HAS-HOLE S1 TWO BACK) (lasthole S1 TWO BACK) (linked S1 nowidth noorient TWO BACK)
    (TEMPERATURE S1 COLD)
    (SHAPE U1 CIRCULAR)
    (SURFACE-CONDITION U1 POLISHED)
    (PAINTED U1 RED)
    (HAS-HOLE U1 THREE FRONT) (lasthole U1 THREE FRONT) (linked U1 nowidth noorient THREE FRONT)
    (TEMPERATURE U1 COLD)
    (SHAPE V1 OBLONG)
    (SURFACE-CONDITION V1 ROUGH)
    (PAINTED V1 RED)
    (HAS-HOLE V1 TWO FRONT) (lasthole V1 TWO FRONT) (linked V1 nowidth noorient TWO FRONT)
    (TEMPERATURE V1 COLD)
    (SHAPE W1 OBLONG)
    (SURFACE-CONDITION W1 SMOOTH)
    (PAINTED W1 BLUE)
    (HAS-HOLE W1 TWO FRONT) (lasthole W1 TWO FRONT) (linked W1 nowidth noorient TWO FRONT)
    (TEMPERATURE W1 COLD)
    (SHAPE Z1 CIRCULAR)
    (SURFACE-CONDITION Z1 POLISHED)
    (PAINTED Z1 BLUE)
    (HAS-HOLE Z1 THREE BACK) (lasthole Z1 THREE BACK) (linked Z1 nowidth noorient THREE BACK)
    (TEMPERATURE Z1 COLD)
    (SHAPE A2 OBLONG)
    (SURFACE-CONDITION A2 ROUGH)
    (PAINTED A2 RED)
    (HAS-HOLE A2 TWO FRONT) (lasthole A2 TWO FRONT) (linked A2 nowidth noorient TWO FRONT)
    (TEMPERATURE A2 COLD)
    (CAN-ORIENT DRILL-PRESS BACK)
    (CAN-ORIENT PUNCH BACK)
    (CAN-ORIENT DRILL-PRESS FRONT)
    (CAN-ORIENT PUNCH FRONT)
    (HAS-PAINT IMMERSION-PAINTER YELLOW)
    (HAS-PAINT SPRAY-PAINTER YELLOW)
    (HAS-PAINT IMMERSION-PAINTER BLUE)
    (HAS-PAINT SPRAY-PAINTER BLUE)
    (HAS-PAINT IMMERSION-PAINTER BLACK)
    (HAS-PAINT SPRAY-PAINTER BLACK)
    (HAS-PAINT IMMERSION-PAINTER RED)
    (HAS-PAINT SPRAY-PAINTER RED)
    (HAS-BIT DRILL-PRESS THREE)
    (HAS-BIT PUNCH THREE)
    (HAS-BIT DRILL-PRESS TWO)
    (HAS-BIT PUNCH TWO)
    (HAS-BIT DRILL-PRESS ONE)
    (HAS-BIT PUNCH ONE)
    (PART A2) (unscheduled A2)
    (PART Z1) (unscheduled Z1)
    (PART W1) (unscheduled W1)
    (PART V1) (unscheduled V1)
    (PART U1) (unscheduled U1)
    (PART S1) (unscheduled S1)
    (PART R1) (unscheduled R1)
    (PART P1) (unscheduled P1)
    (PART Q1) (unscheduled Q1)
    (PART O1) (unscheduled O1)
    (PART N1) (unscheduled N1)
    (PART M1) (unscheduled M1)
    (PART L1) (unscheduled L1)
    (PART K1) (unscheduled K1)
    (PART J1) (unscheduled J1)
    (PART I1) (unscheduled I1)
    (PART H1) (unscheduled H1)
    (PART G1) (unscheduled G1)
    (PART F1) (unscheduled F1)
    (PART E1) (unscheduled E1)
    (PART D1) (unscheduled D1)
    (PART C1) (unscheduled C1)
    (PART B1) (unscheduled B1)
    (PART A1) (unscheduled A1)
    (PART Z0) (unscheduled Z0)
    (PART W0) (unscheduled W0)
    (PART V0) (unscheduled V0)
    (PART U0) (unscheduled U0)
    (PART S0) (unscheduled S0)
    (PART R0) (unscheduled R0)
    (PART P0) (unscheduled P0)
    (PART Q0) (unscheduled Q0)
    (PART O0) (unscheduled O0)
    (PART N0) (unscheduled N0)
    (PART M0) (unscheduled M0)
    (PART L0) (unscheduled L0)
    (PART K0) (unscheduled K0)
    (PART J0) (unscheduled J0)
    (PART I0) (unscheduled I0)
    (PART H0) (unscheduled H0)
    (PART G0) (unscheduled G0)
    (PART F0) (unscheduled F0)
    (PART E0) (unscheduled E0)
    (PART D0) (unscheduled D0)
    (PART C0) (unscheduled C0)
    (PART B0) (unscheduled B0)
    (PART A0) (unscheduled A0)
)
(:goal (and
    (SHAPE E1 CYLINDRICAL)
    (SURFACE-CONDITION I1 POLISHED)
    (SURFACE-CONDITION F0 ROUGH)
    (SHAPE U1 CYLINDRICAL)
    (PAINTED K1 BLACK)
    (SURFACE-CONDITION S1 ROUGH)
    (SURFACE-CONDITION O1 POLISHED)
    (PAINTED R0 BLACK)
    (PAINTED E1 RED)
    (SHAPE W1 CYLINDRICAL)
    (SHAPE Z0 CYLINDRICAL)
    (SHAPE L0 CYLINDRICAL)
    (SHAPE S0 CYLINDRICAL)
    (SURFACE-CONDITION A1 POLISHED)
    (SURFACE-CONDITION D1 SMOOTH)
    (PAINTED C0 BLUE)
    (SURFACE-CONDITION Z0 POLISHED)
    (PAINTED E0 YELLOW)
    (SURFACE-CONDITION P0 POLISHED)
    (SHAPE G1 CYLINDRICAL)
    (SURFACE-CONDITION J1 POLISHED)
    (SURFACE-CONDITION K0 SMOOTH)
    (PAINTED A0 YELLOW)
    (SURFACE-CONDITION U1 ROUGH)
    (PAINTED U1 BLACK)
    (SURFACE-CONDITION B1 ROUGH)
    (SHAPE V1 CYLINDRICAL)
    (PAINTED Z1 YELLOW)
    (SURFACE-CONDITION O0 ROUGH)
    (PAINTED N1 YELLOW)
    (PAINTED U0 YELLOW)
    (SHAPE V0 CYLINDRICAL)
    (SHAPE O1 CYLINDRICAL)
    (PAINTED L0 YELLOW)
    (PAINTED P0 RED)
    (PAINTED C1 YELLOW)
    (SHAPE I1 CYLINDRICAL)
    (PAINTED H0 YELLOW)
    (PAINTED I0 BLUE)
    (SHAPE A2 CYLINDRICAL)
    (SHAPE L1 CYLINDRICAL)
    (SHAPE J1 CYLINDRICAL)
    (PAINTED B1 BLACK)
    (SURFACE-CONDITION S0 POLISHED)
    (SHAPE O0 CYLINDRICAL)
    (PAINTED W0 BLACK)
    (SHAPE E0 CYLINDRICAL)
)))
