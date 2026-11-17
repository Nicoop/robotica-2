% Script para usar CIN_INV_3GDL suponiendo que queremos llegar siempre con
% una orientacion fija, solo ingresar las coordenadas del punto xyz

R_d = [
     0  0  1;
     0  1  0;
    -1  0  0
];

P = [0.04;
     0.02;
     0.0];

T_d = [R_d P;
       0 0 0 1];


q0= [0,0,0] ;
Q = cin_inv_3GDL(R, T_d, q0, true);
T_check = R.fkine(Q)
