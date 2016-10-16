(define (problem schedule-43-9)
(:domain schedule)
(:objects
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
    (SHAPE A0 CIRCULAR)
    (SURFACE-CONDITION A0 POLISHED)
    (PAINTED A0 RED)
    (HAS-HOLE A0 ONE FRONT) (lasthole A0 ONE FRONT) (linked A0 nowidth noorient ONE FRONT)
    (TEMPERATURE A0 COLD)
    (SHAPE B0 CYLINDRICAL)
    (SURFACE-CONDITION B0 POLISHED)
    (PAINTED B0 RED)
    (HAS-HOLE B0 TWO FRONT) (lasthole B0 TWO FRONT) (linked B0 nowidth noorient TWO FRONT)
    (TEMPERATURE B0 COLD)
    (SHAPE C0 CYLINDRICAL)
    (SURFACE-CONDITION C0 ROUGH)
    (PAINTED C0 RED)
    (HAS-HOLE C0 ONE BACK) (lasthole C0 ONE BACK) (linked C0 nowidth noorient ONE BACK)
    (TEMPERATURE C0 COLD)
    (SHAPE D0 CIRCULAR)
    (SURFACE-CONDITION D0 POLISHED)
    (PAINTED D0 RED)
    (HAS-HOLE D0 THREE BACK) (lasthole D0 THREE BACK) (linked D0 nowidth noorient THREE BACK)
    (TEMPERATURE D0 COLD)
    (SHAPE E0 CIRCULAR)
    (SURFACE-CONDITION E0 POLISHED)
    (PAINTED E0 YELLOW)
    (HAS-HOLE E0 TWO FRONT) (lasthole E0 TWO FRONT) (linked E0 nowidth noorient TWO FRONT)
    (TEMPERATURE E0 COLD)
    (SHAPE F0 OBLONG)
    (SURFACE-CONDITION F0 SMOOTH)
    (PAINTED F0 BLUE)
    (HAS-HOLE F0 TWO BACK) (lasthole F0 TWO BACK) (linked F0 nowidth noorient TWO BACK)
    (TEMPERATURE F0 COLD)
    (SHAPE G0 OBLONG)
    (SURFACE-CONDITION G0 POLISHED)
    (PAINTED G0 RED)
    (HAS-HOLE G0 TWO FRONT) (lasthole G0 TWO FRONT) (linked G0 nowidth noorient TWO FRONT)
    (TEMPERATURE G0 COLD)
    (SHAPE H0 OBLONG)
    (SURFACE-CONDITION H0 ROUGH)
    (PAINTED H0 BLACK)
    (HAS-HOLE H0 THREE FRONT) (lasthole H0 THREE FRONT) (linked H0 nowidth noorient THREE FRONT)
    (TEMPERATURE H0 COLD)
    (SHAPE I0 CIRCULAR)
    (SURFACE-CONDITION I0 SMOOTH)
    (PAINTED I0 YELLOW)
    (HAS-HOLE I0 THREE BACK) (lasthole I0 THREE BACK) (linked I0 nowidth noorient THREE BACK)
    (TEMPERATURE I0 COLD)
    (SHAPE J0 CYLINDRICAL)
    (SURFACE-CONDITION J0 POLISHED)
    (PAINTED J0 YELLOW)
    (HAS-HOLE J0 ONE BACK) (lasthole J0 ONE BACK) (linked J0 nowidth noorient ONE BACK)
    (TEMPERATURE J0 COLD)
    (SHAPE K0 CIRCULAR)
    (SURFACE-CONDITION K0 ROUGH)
    (PAINTED K0 BLACK)
    (HAS-HOLE K0 TWO FRONT) (lasthole K0 TWO FRONT) (linked K0 nowidth noorient TWO FRONT)
    (TEMPERATURE K0 COLD)
    (SHAPE L0 CYLINDRICAL)
    (SURFACE-CONDITION L0 SMOOTH)
    (PAINTED L0 BLACK)
    (HAS-HOLE L0 ONE FRONT) (lasthole L0 ONE FRONT) (linked L0 nowidth noorient ONE FRONT)
    (TEMPERATURE L0 COLD)
    (SHAPE M0 CIRCULAR)
    (SURFACE-CONDITION M0 POLISHED)
    (PAINTED M0 YELLOW)
    (HAS-HOLE M0 ONE FRONT) (lasthole M0 ONE FRONT) (linked M0 nowidth noorient ONE FRONT)
    (TEMPERATURE M0 COLD)
    (SHAPE N0 CIRCULAR)
    (SURFACE-CONDITION N0 POLISHED)
    (PAINTED N0 BLACK)
    (HAS-HOLE N0 ONE FRONT) (lasthole N0 ONE FRONT) (linked N0 nowidth noorient ONE FRONT)
    (TEMPERATURE N0 COLD)
    (SHAPE O0 CIRCULAR)
    (SURFACE-CONDITION O0 ROUGH)
    (PAINTED O0 YELLOW)
    (HAS-HOLE O0 ONE FRONT) (lasthole O0 ONE FRONT) (linked O0 nowidth noorient ONE FRONT)
    (TEMPERATURE O0 COLD)
    (SHAPE Q0 CYLINDRICAL)
    (SURFACE-CONDITION Q0 ROUGH)
    (PAINTED Q0 YELLOW)
    (HAS-HOLE Q0 ONE BACK) (lasthole Q0 ONE BACK) (linked Q0 nowidth noorient ONE BACK)
    (TEMPERATURE Q0 COLD)
    (SHAPE P0 CYLINDRICAL)
    (SURFACE-CONDITION P0 POLISHED)
    (PAINTED P0 YELLOW)
    (HAS-HOLE P0 THREE FRONT) (lasthole P0 THREE FRONT) (linked P0 nowidth noorient THREE FRONT)
    (TEMPERATURE P0 COLD)
    (SHAPE R0 CIRCULAR)
    (SURFACE-CONDITION R0 POLISHED)
    (PAINTED R0 YELLOW)
    (HAS-HOLE R0 ONE BACK) (lasthole R0 ONE BACK) (linked R0 nowidth noorient ONE BACK)
    (TEMPERATURE R0 COLD)
    (SHAPE S0 OBLONG)
    (SURFACE-CONDITION S0 POLISHED)
    (PAINTED S0 RED)
    (HAS-HOLE S0 THREE BACK) (lasthole S0 THREE BACK) (linked S0 nowidth noorient THREE BACK)
    (TEMPERATURE S0 COLD)
    (SHAPE U0 CIRCULAR)
    (SURFACE-CONDITION U0 ROUGH)
    (PAINTED U0 BLUE)
    (HAS-HOLE U0 THREE FRONT) (lasthole U0 THREE FRONT) (linked U0 nowidth noorient THREE FRONT)
    (TEMPERATURE U0 COLD)
    (SHAPE V0 CIRCULAR)
    (SURFACE-CONDITION V0 ROUGH)
    (PAINTED V0 RED)
    (HAS-HOLE V0 TWO FRONT) (lasthole V0 TWO FRONT) (linked V0 nowidth noorient TWO FRONT)
    (TEMPERATURE V0 COLD)
    (SHAPE W0 CIRCULAR)
    (SURFACE-CONDITION W0 SMOOTH)
    (PAINTED W0 BLACK)
    (HAS-HOLE W0 THREE BACK) (lasthole W0 THREE BACK) (linked W0 nowidth noorient THREE BACK)
    (TEMPERATURE W0 COLD)
    (SHAPE Z0 CIRCULAR)
    (SURFACE-CONDITION Z0 POLISHED)
    (PAINTED Z0 YELLOW)
    (HAS-HOLE Z0 THREE BACK) (lasthole Z0 THREE BACK) (linked Z0 nowidth noorient THREE BACK)
    (TEMPERATURE Z0 COLD)
    (SHAPE A1 CIRCULAR)
    (SURFACE-CONDITION A1 ROUGH)
    (PAINTED A1 YELLOW)
    (HAS-HOLE A1 ONE BACK) (lasthole A1 ONE BACK) (linked A1 nowidth noorient ONE BACK)
    (TEMPERATURE A1 COLD)
    (SHAPE B1 OBLONG)
    (SURFACE-CONDITION B1 SMOOTH)
    (PAINTED B1 BLUE)
    (HAS-HOLE B1 ONE FRONT) (lasthole B1 ONE FRONT) (linked B1 nowidth noorient ONE FRONT)
    (TEMPERATURE B1 COLD)
    (SHAPE C1 CIRCULAR)
    (SURFACE-CONDITION C1 POLISHED)
    (PAINTED C1 BLACK)
    (HAS-HOLE C1 TWO FRONT) (lasthole C1 TWO FRONT) (linked C1 nowidth noorient TWO FRONT)
    (TEMPERATURE C1 COLD)
    (SHAPE D1 CYLINDRICAL)
    (SURFACE-CONDITION D1 SMOOTH)
    (PAINTED D1 YELLOW)
    (HAS-HOLE D1 ONE BACK) (lasthole D1 ONE BACK) (linked D1 nowidth noorient ONE BACK)
    (TEMPERATURE D1 COLD)
    (SHAPE E1 OBLONG)
    (SURFACE-CONDITION E1 SMOOTH)
    (PAINTED E1 RED)
    (HAS-HOLE E1 ONE BACK) (lasthole E1 ONE BACK) (linked E1 nowidth noorient ONE BACK)
    (TEMPERATURE E1 COLD)
    (SHAPE F1 CYLINDRICAL)
    (SURFACE-CONDITION F1 SMOOTH)
    (PAINTED F1 BLACK)
    (HAS-HOLE F1 THREE FRONT) (lasthole F1 THREE FRONT) (linked F1 nowidth noorient THREE FRONT)
    (TEMPERATURE F1 COLD)
    (SHAPE G1 CIRCULAR)
    (SURFACE-CONDITION G1 ROUGH)
    (PAINTED G1 BLACK)
    (HAS-HOLE G1 ONE FRONT) (lasthole G1 ONE FRONT) (linked G1 nowidth noorient ONE FRONT)
    (TEMPERATURE G1 COLD)
    (SHAPE H1 CIRCULAR)
    (SURFACE-CONDITION H1 POLISHED)
    (PAINTED H1 RED)
    (HAS-HOLE H1 TWO FRONT) (lasthole H1 TWO FRONT) (linked H1 nowidth noorient TWO FRONT)
    (TEMPERATURE H1 COLD)
    (SHAPE I1 OBLONG)
    (SURFACE-CONDITION I1 SMOOTH)
    (PAINTED I1 RED)
    (HAS-HOLE I1 THREE BACK) (lasthole I1 THREE BACK) (linked I1 nowidth noorient THREE BACK)
    (TEMPERATURE I1 COLD)
    (SHAPE J1 OBLONG)
    (SURFACE-CONDITION J1 SMOOTH)
    (PAINTED J1 BLACK)
    (HAS-HOLE J1 TWO FRONT) (lasthole J1 TWO FRONT) (linked J1 nowidth noorient TWO FRONT)
    (TEMPERATURE J1 COLD)
    (SHAPE K1 CIRCULAR)
    (SURFACE-CONDITION K1 SMOOTH)
    (PAINTED K1 BLUE)
    (HAS-HOLE K1 TWO BACK) (lasthole K1 TWO BACK) (linked K1 nowidth noorient TWO BACK)
    (TEMPERATURE K1 COLD)
    (SHAPE L1 CIRCULAR)
    (SURFACE-CONDITION L1 ROUGH)
    (PAINTED L1 YELLOW)
    (HAS-HOLE L1 THREE FRONT) (lasthole L1 THREE FRONT) (linked L1 nowidth noorient THREE FRONT)
    (TEMPERATURE L1 COLD)
    (SHAPE M1 OBLONG)
    (SURFACE-CONDITION M1 ROUGH)
    (PAINTED M1 RED)
    (HAS-HOLE M1 TWO BACK) (lasthole M1 TWO BACK) (linked M1 nowidth noorient TWO BACK)
    (TEMPERATURE M1 COLD)
    (SHAPE N1 CIRCULAR)
    (SURFACE-CONDITION N1 SMOOTH)
    (PAINTED N1 YELLOW)
    (HAS-HOLE N1 THREE FRONT) (lasthole N1 THREE FRONT) (linked N1 nowidth noorient THREE FRONT)
    (TEMPERATURE N1 COLD)
    (SHAPE O1 OBLONG)
    (SURFACE-CONDITION O1 ROUGH)
    (PAINTED O1 YELLOW)
    (HAS-HOLE O1 TWO BACK) (lasthole O1 TWO BACK) (linked O1 nowidth noorient TWO BACK)
    (TEMPERATURE O1 COLD)
    (SHAPE Q1 CIRCULAR)
    (SURFACE-CONDITION Q1 ROUGH)
    (PAINTED Q1 BLACK)
    (HAS-HOLE Q1 TWO FRONT) (lasthole Q1 TWO FRONT) (linked Q1 nowidth noorient TWO FRONT)
    (TEMPERATURE Q1 COLD)
    (SHAPE P1 CIRCULAR)
    (SURFACE-CONDITION P1 POLISHED)
    (PAINTED P1 RED)
    (HAS-HOLE P1 THREE FRONT) (lasthole P1 THREE FRONT) (linked P1 nowidth noorient THREE FRONT)
    (TEMPERATURE P1 COLD)
    (SHAPE R1 CYLINDRICAL)
    (SURFACE-CONDITION R1 POLISHED)
    (PAINTED R1 BLUE)
    (HAS-HOLE R1 TWO BACK) (lasthole R1 TWO BACK) (linked R1 nowidth noorient TWO BACK)
    (TEMPERATURE R1 COLD)
    (SHAPE S1 CIRCULAR)
    (SURFACE-CONDITION S1 SMOOTH)
    (PAINTED S1 YELLOW)
    (HAS-HOLE S1 TWO FRONT) (lasthole S1 TWO FRONT) (linked S1 nowidth noorient TWO FRONT)
    (TEMPERATURE S1 COLD)
    (SHAPE U1 CYLINDRICAL)
    (SURFACE-CONDITION U1 ROUGH)
    (PAINTED U1 RED)
    (HAS-HOLE U1 THREE FRONT) (lasthole U1 THREE FRONT) (linked U1 nowidth noorient THREE FRONT)
    (TEMPERATURE U1 COLD)
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
    (PAINTED O0 BLUE)
    (SURFACE-CONDITION N0 SMOOTH)
    (PAINTED G0 BLUE)
    (SHAPE R0 CYLINDRICAL)
    (SURFACE-CONDITION J1 ROUGH)
    (SURFACE-CONDITION O1 POLISHED)
    (SHAPE O0 CYLINDRICAL)
    (PAINTED N1 BLUE)
    (PAINTED J0 BLACK)
    (SHAPE U0 CYLINDRICAL)
    (SURFACE-CONDITION N1 ROUGH)
    (PAINTED Q1 RED)
    (SHAPE J1 CYLINDRICAL)
    (SHAPE P1 CYLINDRICAL)
    (SURFACE-CONDITION G1 POLISHED)
    (PAINTED A1 RED)
    (PAINTED K1 RED)
    (PAINTED R1 RED)
    (SURFACE-CONDITION F0 ROUGH)
    (SURFACE-CONDITION L1 POLISHED)
    (SURFACE-CONDITION Q1 SMOOTH)
    (SURFACE-CONDITION H1 SMOOTH)
    (SHAPE A0 CYLINDRICAL)
    (SURFACE-CONDITION V0 SMOOTH)
    (PAINTED S0 BLUE)
    (SHAPE E0 CYLINDRICAL)
    (PAINTED A0 YELLOW)
    (SURFACE-CONDITION I0 POLISHED)
    (PAINTED D0 BLUE)
    (PAINTED J1 YELLOW)
    (PAINTED L1 RED)
    (PAINTED O1 RED)
    (SHAPE C1 CYLINDRICAL)
    (SHAPE S0 CYLINDRICAL)
    (SURFACE-CONDITION Q0 SMOOTH)
    (SURFACE-CONDITION P0 SMOOTH)
    (PAINTED M0 RED)
    (SHAPE O1 CYLINDRICAL)
    (SURFACE-CONDITION H0 SMOOTH)
    (SURFACE-CONDITION A0 SMOOTH)
    (PAINTED F0 BLACK)
    (SHAPE I1 CYLINDRICAL)
    (SURFACE-CONDITION Z0 SMOOTH)
)))
