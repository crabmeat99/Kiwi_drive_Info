% DATA = DOHVATI Dohvatipodatke i ucitaj ih u varijablu DATA
%   Sljedece naredbe naredbom scp prebacuju matrice podatke s Taraget
%   računala u ../data direktorij nadzornog racunala i ucitavaju ih u
%   zadanu workspace varijablu kao strukturu

function data= dohvati()

data_path = './data';
unix(sprintf('scp root@target:q.mat %s/q.mat', data_path))
unix(sprintf('scp root@target:dq.mat %s/dq.mat', data_path));
unix(sprintf('scp root@target:u.mat %s/u.mat',data_path))
unix(sprintf('scp root@target:q_ref.mat %s/q_ref.mat', data_path))
unix(sprintf('scp root@target:dq_ref.mat %s/dq_ref.mat', data_path));

load(sprintf('%s/q.mat',data_path));
load(sprintf('%s/dq.mat', data_path));
load(sprintf('%s/u.mat', data_path));
load(sprintf('%s/q_ref.mat', data_path));
load(sprintf('%s/dq_ref.mat', data_path));

data.t = q(1,:)';
data.q1 = q(2,:)';
data.q1_ref = q_ref(2,:)';
data.dq1 = dq(2,:)';
data.dq1_ref = dq_ref(2,:)';
data.u1 = u(2,:)';

end