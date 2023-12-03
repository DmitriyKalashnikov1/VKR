Ce = 0.153;
Cm = 0.156;
J =  ureal('J', 1.22e-4, 'Percentage', 20);
Ry = ureal('Ry', 0.34, 'Percentage', 20);
Ly = ureal('Ly', 1e-4, 'Percentage', 20);
Mc = 0.1;
%Mc = ureal('Mc', 0.1, 'Percentage', 10);

Wdc = tf(1, [(Ly*J)/Cm (Ry*J)/Cm Ce]);
Wif = Mc*tf([Ly/Cm Ry/Cm], [(Ly*J)/Cm (Ry*J)/Cm Ce]);
Wif.OutputName = 'InputForce';
Wif.InputName = 'Speed';


sm1 = sumblk('e = SpeedRef - Speed');

C = tunablePID('C','pid');
C.InputName = 'e';
C.OutputName = 'uc';


Wdc.InputName = 'uc';
Wdc.OutputName = 'omega1';

sm2 = sumblk('Speed = omega1 - InputForce');

figure(1);
step(Wdc,getNominal(Wdc),0.1);
legend('Sampled uncertainty','Nominal');

opt1 = connectOptions('Simplify', false);

T = connect(C, Wdc, Wif, sm1, sm2, 'SpeedRef', 'Speed', opt1);



R = TuningGoal.StepResp('SpeedRef', 'Speed', 0.2, 0);

figure(2);
viewGoal(R);

opt = systuneOptions('RandomStart',2);

rng(0), [CL,fSoft] = systune(T, R ,opt);

showTunable(CL);

figure(3);
viewGoal(R, CL);