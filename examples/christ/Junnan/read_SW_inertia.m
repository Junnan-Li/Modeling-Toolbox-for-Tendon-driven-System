% list = dir(fullfile(pwd, 'SW_inertia'));
% frame_index_order = {'T_0_0', 'T_0_sh1', 'T_0_sh2','T_0_sh3', 'T_0_el', 'T_0_fa', 'T_0_wj1', 'T_0_wj2'};

%   m_p_out(i,:) = str_(i).m_vec_SI;
%         rSges_p_out(i,:) = str_(i).r_COM_SI ;
%         Ifges_p_out(i,:) = str_(i).J_KS_SI;

function [m_p_out, rSges_p_out, mrSges_p_out, Ifges_p_out, Icges_p_out, str_] = read_SW_inertia(path_, frame_index_order, m_p, rSges_p, Ifges_p)



% Alexander Toedtheide

list = dir(fullfile(path_, '*.txt'));

k = 1;
for i = 1:length(list)
    
    
    text = fileread(fullfile(path_, list(i).name));
    disp(list(i).name)
    
    
    %% find limiters
    newStr = splitlines(text);
    indices = find(cellfun(@isempty,newStr));
    
    %% name
    index__ = find_str_in_cell(newStr, 'Koordinatensystem:');
    strings___ = strsplit(newStr{index__});
    str_(k).name = strings___{end};
    
    %% mass
    index__ = find_str_in_cell(newStr, 'Masse = ');
    mass_g_raw = strsplit(newStr{index__});
    mass_g = str2num(mass_g_raw{3});
    str_(k).m_vec_SI = mass_g/1000;
    
    %% Center of mass
    index__ = find_str_in_cell(newStr, 'X = ');
    com1_mm_raw = strsplit(newStr{index__});
    com2_mm_raw = strsplit(newStr{index__+1});
    com3_mm_raw = strsplit(newStr{index__+2});
    rx_mm = str2num(com1_mm_raw{4});
    ry_mm = str2num(com2_mm_raw{4});
    rz_mm = str2num(com3_mm_raw{4});
    
    str_(k).r_COM_SI = [rx_mm, ry_mm, rz_mm]/1000;
    
    
    %% Inertia matrix

    [J_matrix_C, J_matrix_C_vec] = inertiaFormatFromFile(newStr, 'Lxx =');
    % Ixx, Iyy, Izz, Ixy, Ixz, Iyz
    str_(k).J_C_CI = J_matrix_C_vec*1e-9;
        
    [J_matrix_F, J_matrix_F_vec] = inertiaFormatFromFile(newStr, 'Ixx =');
    str_(k).J_KF_FI = J_matrix_F_vec*1e-9;
    
%     str_(k).J_KF_FI = J_matrix_F_vec*1e-9;

    J_F_check_1 = inertia_steiner(inertiavector2matrix(str_(k).J_C_CI),  str_(k).r_COM_SI', str_(k).m_vec_SI);
    
    if any(any(J_matrix_F*1e-9 - J_F_check_1> 100 ))
       warning('Difference between inertia matrices F and C. There seems to be an error in the txt data or SW') 
    else 
        disp('Inertia matrices C and F match')
    end

    k = k + 1;
end

try
load(fullfile(path_, 'mech_param'))
if ~exist('m_p') == 1
    m_p = m;
end

if ~exist('rSges_p') == 1
    rSges_p = rSges;
end

if ~exist('Ifges_p') == 1
    Ifges_p = Ifges;
end

if ~exist('Icges_p') == 1
    Icges_p = Icges;
end


catch
warning('place a file mech_param.mat to the same folder as the SW output files (compatible to the maple toolbox) containing the parameters m_p, rSges_p, Ifges_p or m, rSges, Ifges')    
end

%% find fieldname
for i = 1:length(frame_index_order)

    index = find(strcmp({str_.name}, frame_index_order{i})==1);
    if isempty(index)
        m_p_out(i,:) = m_p(i);
        rSges_p_out(i,:) = rSges_p(i,:);
        Ifges_p_out(i,:) = Ifges_p(i,:);
        Icges_p_out(i,:) = Icges_p(i,:);
    else
        str_(index).name;
        m_p_out(i,:) = str_(index).m_vec_SI;
        rSges_p_out(i,:) = str_(index).r_COM_SI ;
        Ifges_p_out(i,:) = str_(index).J_KF_FI;
        Icges_p_out(i,:) =  str_(index).J_C_CI;
    end


end
index__ = 1;

mrSges_p_out = m_p_out.*rSges_p_out;

for i = 1:size(Icges_p,1)
MMM = inertiavector2matrix(Icges_p(i, :));

if any(eig(MMM )) < 0
    error('Inertia matrices not corect eig(I) < 0')
end


end
end

function i = find_str_in_cell(newStr, search_string)
a = strfind(newStr, search_string);
i = find(~cellfun(@isempty,a'));

end

function ICges_out = interta_F_to_C(Ifges_in, rSges, m)


%% Trägheitstensor um das Körperkoordinatensystem
ICges_out = NaN(length(m),6);
for i = 1:length(m)
  % Trägheitstensor um den Schwerpunkt
  Ifges = inertiavector2matrix(Ifges_in(i,:));
  
  % Steinerscher-Verschiebungssatz: Trägheitstensor um Koordinatensystem
  ICges = inertia_steiner_back(Ifges, rSges(i,:)', m(i));

  % Ausgabe zusammenstellen
  ICges_out(i,:) = inertiamatrix2vector(ICges);
  
end


end

function [J_matrix, J_matrix_vec] = inertiaFormatFromFile(newStr, Identifier)

% Identifier'Ixx ='

index__ = find_str_in_cell(newStr, Identifier);
    I1_gmm2_raw = strsplit(newStr{index__});
    I2_gmm2_raw = strsplit(newStr{index__+1});
    I3_gmm2_raw = strsplit(newStr{index__+2});
    
    Ixx_gmm2 = str2num(I1_gmm2_raw{4});
    Ixy_gmm2 = str2num(I1_gmm2_raw{7});
    Ixz_gmm2 = str2num(I1_gmm2_raw{10});
    
    Iyx_gmm2 = str2num(I2_gmm2_raw{4});
    Iyy_gmm2 = str2num(I2_gmm2_raw{7});
    Iyz_gmm2 = str2num(I2_gmm2_raw{10});
    
    Izx_gmm2 = str2num(I3_gmm2_raw{4});
    Izy_gmm2 = str2num(I3_gmm2_raw{7});
    Izz_gmm2 = str2num(I3_gmm2_raw{10});
    
    J_matrix = ([Ixx_gmm2 Ixy_gmm2 Ixz_gmm2; Iyx_gmm2 Iyy_gmm2 Iyz_gmm2; Izx_gmm2 Izy_gmm2 Izz_gmm2]);

    J_matrix_vec = inertiamatrix2vector(J_matrix);
end

