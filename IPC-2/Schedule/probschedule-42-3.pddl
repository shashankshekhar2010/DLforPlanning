(define (problem schedule-42-3)
(:domain schedule)
(:objects
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
    (SHAPE A0 CIRCULAR)
    (SURFACE-CONDITION A0 SMOOTH)
    (PAINTED A0 YELLOW)
    (HAS-HOLE A0 TWO FRONT) (lasthole A0 TWO FRONT) (linked A0 nowidth noorient TWO FRONT)
    (TEMPERATURE A0 COLD)
    (SHAPE B0 CIRCULAR)
    (SURFACE-CONDITION B0 SMOOTH)
    (PAINTED B0 BLUE)
    (HAS-HOLE B0 ONE BACK) (lasthole B0 ONE BACK) (linked B0 nowidth noorient ONE BACK)
    (TEMPERATURE B0 COLD)
    (SHAPE C0 CYLINDRICAL)
    (SURFACE-CONDITION C0 SMOOTH)
    (PAINTED C0 BLACK)
    (HAS-HOLE C0 ONE FRONT) (lasthole C0 ONE FRONT) (linked C0 nowidth noorient ONE FRONT)
    (TEMPERATURE C0 COLD)
    (SHAPE D0 CIRCULAR)
    (SURFACE-CONDITION D0 ROUGH)
    (PAINTED D0 BLUE)
    (HAS-HOLE D0 ONE BACK) (lasthole D0 ONE BACK) (linked D0 nowidth noorient ONE BACK)
    (TEMPERATURE D0 COLD)
    (SHAPE E0 CYLINDRICAL)
    (SURFACE-CONDITION E0 SMOOTH)
    (PAINTED E0 YELLOW)
    (HAS-HOLE E0 TWO BACK) (lasthole E0 TWO BACK) (linked E0 nowidth noorient TWO BACK)
    (TEMPERATURE E0 COLD)
    (SHAPE F0 OBLONG)
    (SURFACE-CONDITION F0 SMOOTH)
    (PAINTED F0 BLUE)
    (HAS-HOLE F0 THREE FRONT) (lasthole F0 THREE FRONT) (linked F0 nowidth noorient THREE FRONT)
    (TEMPERATURE F0 COLD)
    (SHAPE G0 OBLONG)
    (SURFACE-CONDITION G0 ROUGH)
    (PAINTED G0 RED)
    (HAS-HOLE G0 ONE FRONT) (lasthole G0 ONE FRONT) (linked G0 nowidth noorient ONE FRONT)
    (TEMPERATURE G0 COLD)
    (SHAPE H0 CIRCULAR)
    (SURFACE-CONDITION H0 SMOOTH)
    (PAINTED H0 RED)
    (HAS-HOLE H0 ONE FRONT) (lasthole H0 ONE FRONT) (linked H0 nowidth noorient ONE FRONT)
    (TEMPERATURE H0 COLD)
    (SHAPE I0 CYLINDRICAL)
    (SURFACE-CONDITION I0 ROUGH)
    (PAINTED I0 BLACK)
    (HAS-HOLE I0 TWO BACK) (lasthole I0 TWO BACK) (linked I0 nowidth noorient TWO BACK)
    (TEMPERATURE I0 COLD)
    (SHAPE J0 CIRCULAR)
    (SURFACE-CONDITION J0 POLISHED)
    (PAINTED J0 YELLOW)
    (HAS-HOLE J0 ONE FRONT) (lasthole J0 ONE FRONT) (linked J0 nowidth noorient ONE FRONT)
    (TEMPERATURE J0 COLD)
    (SHAPE K0 CYLINDRICAL)
    (SURFACE-CONDITION K0 POLISHED)
    (PAINTED K0 RED)
    (HAS-HOLE K0 THREE FRONT) (lasthole K0 THREE FRONT) (linked K0 nowidth noorient THREE FRONT)
    (TEMPERATURE K0 COLD)
    (SHAPE L0 OBLONG)
    (SURFACE-CONDITION L0 SMOOTH)
    (PAINTED L0 BLUE)
    (HAS-HOLE L0 TWO FRONT) (lasthole L0 TWO FRONT) (linked L0 nowidth noorient TWO FRONT)
    (TEMPERATURE L0 COLD)
    (SHAPE M0 CIRCULAR)
    (SURFACE-CONDITION M0 ROUGH)
    (PAINTED M0 RED)
    (HAS-HOLE M0 THREE BACK) (lasthole M0 THREE BACK) (linked M0 nowidth noorient THREE BACK)
    (TEMPERATURE M0 COLD)
    (SHAPE N0 CIRCULAR)
    (SURFACE-CONDITION N0 POLISHED)
    (PAINTED N0 YELLOW)
    (HAS-HOLE N0 TWO FRONT) (lasthole N0 TWO FRONT) (linked N0 nowidth noorient TWO FRONT)
    (TEMPERATURE N0 COLD)
    (SHAPE O0 CIRCULAR)
    (SURFACE-CONDITION O0 ROUGH)
    (PAINTED O0 BLACK)
    (HAS-HOLE O0 THREE FRONT) (lasthole O0 THREE FRONT) (linked O0 nowidth noorient THREE FRONT)
    (TEMPERATURE O0 COLD)
    (SHAPE Q0 CIRCULAR)
    (SURFACE-CONDITION Q0 ROUGH)
    (PAINTED Q0 BLACK)
    (HAS-HOLE Q0 ONE FRONT) (lasthole Q0 ONE FRONT) (linked Q0 nowidth noorient ONE FRONT)
    (TEMPERATURE Q0 COLD)
    (SHAPE P0 CYLINDRICAL)
    (SURFACE-CONDITION P0 SMOOTH)
    (PAINTED P0 BLUE)
    (HAS-HOLE P0 TWO FRONT) (lasthole P0 TWO FRONT) (linked P0 nowidth noorient TWO FRONT)
    (TEMPERATURE P0 COLD)
    (SHAPE R0 OBLONG)
    (SURFACE-CONDITION R0 POLISHED)
    (PAINTED R0 BLUE)
    (HAS-HOLE R0 THREE FRONT) (lasthole R0 THREE FRONT) (linked R0 nowidth noorient THREE FRONT)
    (TEMPERATURE R0 COLD)
    (SHAPE S0 OBLONG)
    (SURFACE-CONDITION S0 ROUGH)
    (PAINTED S0 YELLOW)
    (HAS-HOLE S0 THREE FRONT) (lasthole S0 THREE FRONT) (linked S0 nowidth noorient THREE FRONT)
    (TEMPERATURE S0 COLD)
    (SHAPE U0 OBLONG)
    (SURFACE-CONDITION U0 SMOOTH)
    (PAINTED U0 YELLOW)
    (HAS-HOLE U0 THREE FRONT) (lasthole U0 THREE FRONT) (linked U0 nowidth noorient THREE FRONT)
    (TEMPERATURE U0 COLD)
    (SHAPE V0 OBLONG)
    (SURFACE-CONDITION V0 SMOOTH)
    (PAINTED V0 YELLOW)
    (HAS-HOLE V0 TWO BACK) (lasthole V0 TWO BACK) (linked V0 nowidth noorient TWO BACK)
    (TEMPERATURE V0 COLD)
    (SHAPE W0 OBLONG)
    (SURFACE-CONDITION W0 SMOOTH)
    (PAINTED W0 BLACK)
    (HAS-HOLE W0 THREE BACK) (lasthole W0 THREE BACK) (linked W0 nowidth noorient THREE BACK)
    (TEMPERATURE W0 COLD)
    (SHAPE Z0 CYLINDRICAL)
    (SURFACE-CONDITION Z0 SMOOTH)
    (PAINTED Z0 YELLOW)
    (HAS-HOLE Z0 ONE FRONT) (lasthole Z0 ONE FRONT) (linked Z0 nowidth noorient ONE FRONT)
    (TEMPERATURE Z0 COLD)
    (SHAPE A1 CYLINDRICAL)
    (SURFACE-CONDITION A1 POLISHED)
    (PAINTED A1 YELLOW)
    (HAS-HOLE A1 TWO BACK) (lasthole A1 TWO BACK) (linked A1 nowidth noorient TWO BACK)
    (TEMPERATURE A1 COLD)
    (SHAPE B1 CIRCULAR)
    (SURFACE-CONDITION B1 POLISHED)
    (PAINTED B1 BLUE)
    (HAS-HOLE B1 THREE BACK) (lasthole B1 THREE BACK) (linked B1 nowidth noorient THREE BACK)
    (TEMPERATURE B1 COLD)
    (SHAPE C1 CIRCULAR)
    (SURFACE-CONDITION C1 ROUGH)
    (PAINTED C1 RED)
    (HAS-HOLE C1 TWO BACK) (lasthole C1 TWO BACK) (linked C1 nowidth noorient TWO BACK)
    (TEMPERATURE C1 COLD)
    (SHAPE D1 CIRCULAR)
    (SURFACE-CONDITION D1 ROUGH)
    (PAINTED D1 RED)
    (HAS-HOLE D1 THREE BACK) (lasthole D1 THREE BACK) (linked D1 nowidth noorient THREE BACK)
    (TEMPERATURE D1 COLD)
    (SHAPE E1 OBLONG)
    (SURFACE-CONDITION E1 SMOOTH)
    (PAINTED E1 YELLOW)
    (HAS-HOLE E1 ONE BACK) (lasthole E1 ONE BACK) (linked E1 nowidth noorient ONE BACK)
    (TEMPERATURE E1 COLD)
    (SHAPE F1 CIRCULAR)
    (SURFACE-CONDITION F1 POLISHED)
    (PAINTED F1 RED)
    (HAS-HOLE F1 TWO BACK) (lasthole F1 TWO BACK) (linked F1 nowidth noorient TWO BACK)
    (TEMPERATURE F1 COLD)
    (SHAPE G1 CYLINDRICAL)
    (SURFACE-CONDITION G1 ROUGH)
    (PAINTED G1 BLACK)
    (HAS-HOLE G1 ONE BACK) (lasthole G1 ONE BACK) (linked G1 nowidth noorient ONE BACK)
    (TEMPERATURE G1 COLD)
    (SHAPE H1 CIRCULAR)
    (SURFACE-CONDITION H1 ROUGH)
    (PAINTED H1 BLACK)
    (HAS-HOLE H1 TWO FRONT) (lasthole H1 TWO FRONT) (linked H1 nowidth noorient TWO FRONT)
    (TEMPERATURE H1 COLD)
    (SHAPE I1 OBLONG)
    (SURFACE-CONDITION I1 ROUGH)
    (PAINTED I1 YELLOW)
    (HAS-HOLE I1 TWO BACK) (lasthole I1 TWO BACK) (linked I1 nowidth noorient TWO BACK)
    (TEMPERATURE I1 COLD)
    (SHAPE J1 CIRCULAR)
    (SURFACE-CONDITION J1 ROUGH)
    (PAINTED J1 BLACK)
    (HAS-HOLE J1 ONE BACK) (lasthole J1 ONE BACK) (linked J1 nowidth noorient ONE BACK)
    (TEMPERATURE J1 COLD)
    (SHAPE K1 OBLONG)
    (SURFACE-CONDITION K1 SMOOTH)
    (PAINTED K1 RED)
    (HAS-HOLE K1 ONE BACK) (lasthole K1 ONE BACK) (linked K1 nowidth noorient ONE BACK)
    (TEMPERATURE K1 COLD)
    (SHAPE L1 CYLINDRICAL)
    (SURFACE-CONDITION L1 SMOOTH)
    (PAINTED L1 YELLOW)
    (HAS-HOLE L1 TWO BACK) (lasthole L1 TWO BACK) (linked L1 nowidth noorient TWO BACK)
    (TEMPERATURE L1 COLD)
    (SHAPE M1 OBLONG)
    (SURFACE-CONDITION M1 ROUGH)
    (PAINTED M1 BLUE)
    (HAS-HOLE M1 THREE FRONT) (lasthole M1 THREE FRONT) (linked M1 nowidth noorient THREE FRONT)
    (TEMPERATURE M1 COLD)
    (SHAPE N1 OBLONG)
    (SURFACE-CONDITION N1 SMOOTH)
    (PAINTED N1 BLUE)
    (HAS-HOLE N1 THREE FRONT) (lasthole N1 THREE FRONT) (linked N1 nowidth noorient THREE FRONT)
    (TEMPERATURE N1 COLD)
    (SHAPE O1 CYLINDRICAL)
    (SURFACE-CONDITION O1 ROUGH)
    (PAINTED O1 BLACK)
    (HAS-HOLE O1 THREE FRONT) (lasthole O1 THREE FRONT) (linked O1 nowidth noorient THREE FRONT)
    (TEMPERATURE O1 COLD)
    (SHAPE Q1 CYLINDRICAL)
    (SURFACE-CONDITION Q1 POLISHED)
    (PAINTED Q1 YELLOW)
    (HAS-HOLE Q1 TWO FRONT) (lasthole Q1 TWO FRONT) (linked Q1 nowidth noorient TWO FRONT)
    (TEMPERATURE Q1 COLD)
    (SHAPE P1 CIRCULAR)
    (SURFACE-CONDITION P1 SMOOTH)
    (PAINTED P1 YELLOW)
    (HAS-HOLE P1 ONE BACK) (lasthole P1 ONE BACK) (linked P1 nowidth noorient ONE BACK)
    (TEMPERATURE P1 COLD)
    (SHAPE R1 CIRCULAR)
    (SURFACE-CONDITION R1 SMOOTH)
    (PAINTED R1 RED)
    (HAS-HOLE R1 THREE FRONT) (lasthole R1 THREE FRONT) (linked R1 nowidth noorient THREE FRONT)
    (TEMPERATURE R1 COLD)
    (SHAPE S1 OBLONG)
    (SURFACE-CONDITION S1 ROUGH)
    (PAINTED S1 YELLOW)
    (HAS-HOLE S1 ONE FRONT) (lasthole S1 ONE FRONT) (linked S1 nowidth noorient ONE FRONT)
    (TEMPERATURE S1 COLD)
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
    (SHAPE N0 CYLINDRICAL)
    (PAINTED O1 YELLOW)
    (SURFACE-CONDITION P1 POLISHED)
    (SURFACE-CONDITION R1 POLISHED)
    (PAINTED J1 BLUE)
    (SHAPE V0 CYLINDRICAL)
    (PAINTED S1 BLUE)
    (SHAPE C1 CYLINDRICAL)
    (SURFACE-CONDITION Q0 POLISHED)
    (SHAPE K1 CYLINDRICAL)
    (SHAPE O0 CYLINDRICAL)
    (SHAPE R1 CYLINDRICAL)
    (SURFACE-CONDITION K1 POLISHED)
    (SURFACE-CONDITION C1 POLISHED)
    (PAINTED E0 BLACK)
    (SHAPE Q0 CYLINDRICAL)
    (SURFACE-CONDITION C0 ROUGH)
    (SHAPE J0 CYLINDRICAL)
    (SHAPE E1 CYLINDRICAL)
    (PAINTED K0 BLUE)
    (SHAPE H1 CYLINDRICAL)
    (PAINTED B0 RED)
    (SURFACE-CONDITION Z0 POLISHED)
    (SURFACE-CONDITION F0 POLISHED)
    (SHAPE N1 CYLINDRICAL)
    (SHAPE G0 CYLINDRICAL)
    (PAINTED D1 YELLOW)
    (PAINTED K1 BLACK)
    (PAINTED L1 BLACK)
    (PAINTED I0 BLUE)
    (PAINTED O0 RED)
    (PAINTED E1 BLUE)
    (SHAPE B1 CYLINDRICAL)
    (PAINTED Q0 RED)
    (PAINTED A0 BLUE)
    (SHAPE J1 CYLINDRICAL)
    (PAINTED R0 RED)
    (SURFACE-CONDITION E1 ROUGH)
    (SHAPE I1 CYLINDRICAL)
    (SHAPE R0 CYLINDRICAL)
    (SHAPE L0 CYLINDRICAL)
    (SHAPE F1 CYLINDRICAL)
)))
