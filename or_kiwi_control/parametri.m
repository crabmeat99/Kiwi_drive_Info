%nazivni moment tromosti zglobova
Jn=[1.8e-4,0.45e-4,0,0];

%Enkoderi
Encoder_counts=[1000; 2000; 2000; 2000];
%Prijenos
Prijenosni_omjeri=[1/160;1/160;4/10*1e-3;12.36/41];
%Motori
Ucc=[24,24,24,24];          %nominalni napon[V]
wnazivno=[2700,2700,5670,5670];   %nominalna brzina[min^-1]
%Iamax=[20,20,0.774,0.774];   %maksimalna struja [A]- specifikacije 
Iamax=[2.5,2.5,1,1];   %maksimalna struja [A]- parametri 
Ra=[2.7,2.7,8,8];           %otpor armature [ohm]
Ka=1./Ra;
La=[3,3,1.54,1.54]*1e-3;    %induktivitet armature [mH]
Ta=La./Ra;
K=[0.11,0.11,0.0394,0.0394];
Jm=[100,100,2.63,2.63]*1e-6;%moment tromosti motora
Bm=[80,80,0,0]*1e-6;        %koeficijent viskoznog otpora motora

%% Parametri regulatora
Ts=8e-3;        %vrijeme uzorkovanja računala
Ju=Jm+Jn;       %ukupni moment tromosti
Krq = 1;
Kr = 5;
Ti = 0.06;