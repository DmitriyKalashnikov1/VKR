Ce = 0.058;
Cm = 0.057;
J = 8.4e-5; %ureal('J', 8.4e-5, 'Percentage', 20);
Ry = ureal('Ry', 0.71, 'Percentage', 20);
Ly = ureal('Ly', 8.6e-4, 'Percentage', 20);
Mc = 0.1;
%Mc = ureal('Mc', 0.1, 'Percentage', 10);

Wdc = tf(1, [(Ly*J)/Cm (Ry*J)/Cm Ce]);
Wif = Mc*tf([Ly/Cm Ry/Cm], [(Ly*J)/Cm (Ry*J)/Cm Ce]);
Wif.OutputName = 'InputForce';
Wif.InputName = 'Angle';


sm1 = sumblk('e = AngleRef - Angle');

C = tunablePID('C','pid');
C.InputName = 'e';
C.OutputName = 'uc';


Wdc.InputName = 'uc';
Wdc.OutputName = 'omega1';

sm2 = sumblk('Speed = omega1 - InputForce');

integrator = tf(1, [1, 0]);
integrator.InputName = "Speed";
integrator.OutputName = "Angle";

figure(1);
step(Wdc,getNominal(Wdc),0.1);
legend('Sampled uncertainty','Nominal');

opt1 = connectOptions('Simplify', false);

T = connect(C, Wdc, Wif, sm1, sm2, integrator, 'AngleRef', 'Angle', opt1);



R = TuningGoal.StepResp('AngleRef', 'Angle', 0.2, 0);

figure(2);
viewGoal(R);

opt = systuneOptions('RandomStart',2);

rng(0), [CL,fSoft] = systune(T, R ,opt);

showTunable(CL);

figure(3);
viewGoal(R, CL);