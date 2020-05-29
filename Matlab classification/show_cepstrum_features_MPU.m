function show_cepstrum_features_MPU(data, num_coeffs, num_bands, win_len, ...
                                                    overlap_len, min_freq, max_freq)
  %% Useful info
  % Shows computed cepstrum features and energy of signal from chosen data
  % record - for 6 axis of MPU sensor
  %
  % data        -  one record of the dataset (about running, walking, etc) of special format
  % num_coeffs  - number of xfcc coefficients that are needed to be computed
  %
  % Output is the data table, that can be used for ML-Classifier training
  %
  % Example of call: 
  %     load measurement_data.mat;
  %     dat1 = data{1,1};
  %     show_cepstrum_features_MPU(dat1, 10, 40, 3.0, 2.0, 0.5, 30);
  
  
  %% Arguments check 
  if ~exist('data','var') || ~exist('num_coeffs','var') || ~exist('num_bands','var') || ...
      ~exist('win_len','var') || ~exist('overlap_len','var') || ...
      ~exist('min_freq','var') || ~exist('max_freq','var')
    fprintf("XFCC_feature_extract()# Error: not sufficient ammount of arguments\n");
    return;
  end

  
  
  %% Feature extraction settings
  numCoeffs= num_coeffs;       % number of cepstral coefficints to get at the end
  numBands= num_bands;
  
  minFrequency= min_freq;
  maxFrequency= max_freq;
  
  windowLength=  win_len;         % in s, should not be shorter than 1/minFrequency
  overlapLength= overlap_len;     % in s, should be shorter than windowLength  
  
  mpuOrigFrequency= 50;          % Hz
  fs= 10000;                      % Hz; because mfcc() supposed to be used with speech signal around 10kHz
  
  % compute band edges for cepstral  
  bandEdges= linspace(minFrequency,maxFrequency,numBands);

  % compute window and overlap length in samples (not in sec.)
  WL= round(fs*windowLength);
  OL= round(fs*overlapLength);  
    
  
  
      %% Compute XFCC coefficents based on MPU accelerometer data
      
      % Mpu accl x-axis
      dataSeries = data.mpu_data(data.mpu_gross(1):data.mpu_gross(2),1);     
      xfcc_mpu_ax = XFCC_one_dataSeries(dataSeries, numCoeffs, mpuOrigFrequency, fs, bandEdges, WL, OL);  

      % Mpu accl y-axis
      dataSeries = data.mpu_data(data.mpu_gross(1):data.mpu_gross(2),2);     
      xfcc_mpu_ay = XFCC_one_dataSeries(dataSeries, numCoeffs, mpuOrigFrequency, fs, bandEdges, WL, OL); 

      % Mpu accl z-axis
      dataSeries = data.mpu_data(data.mpu_gross(1):data.mpu_gross(2),3);     
      xfcc_mpu_az = XFCC_one_dataSeries(dataSeries, numCoeffs, mpuOrigFrequency, fs, bandEdges, WL, OL); 

      % Build variables names for this part of data
      xfcc_mpu_ax_coeffNames = gen_table_variables_names('xfcc_mpu_ax', numCoeffs);
      xfcc_mpu_ay_coeffNames = gen_table_variables_names('xfcc_mpu_ay', numCoeffs);
      xfcc_mpu_az_coeffNames = gen_table_variables_names('xfcc_mpu_az', numCoeffs);

      %% Compute XFCC coefficents based on MPU gyroscope data
      
      % Mpu gyro x-axis
      dataSeries = data.mpu_data(data.mpu_gross(1):data.mpu_gross(2),5);     
      xfcc_mpu_gx = XFCC_one_dataSeries(dataSeries, numCoeffs, mpuOrigFrequency, fs, bandEdges, WL, OL); 

      % Mpu gyro y-axis
      dataSeries = data.mpu_data(data.mpu_gross(1):data.mpu_gross(2),6);     
      xfcc_mpu_gy = XFCC_one_dataSeries(dataSeries, numCoeffs, mpuOrigFrequency, fs, bandEdges, WL, OL); 

      % Mpu gyro z-axis
      dataSeries = data.mpu_data(data.mpu_gross(1):data.mpu_gross(2),7);     
      xfcc_mpu_gz = XFCC_one_dataSeries(dataSeries, numCoeffs, mpuOrigFrequency, fs, bandEdges, WL, OL); 

      % Build variables names for this part of data
      xfcc_mpu_gx_coeffNames = gen_table_variables_names('xfcc_mpu_gx', numCoeffs);
      xfcc_mpu_gy_coeffNames = gen_table_variables_names('xfcc_mpu_gy', numCoeffs);
      xfcc_mpu_gz_coeffNames = gen_table_variables_names('xfcc_mpu_gz', numCoeffs);
 
  
  %% Build XFCC features table with activity labels
  
      
      % Build variable names and activity labels array
      xfcc_array_len = size(xfcc_mpu_ax, 1);
      xfcc_labels = repelem(string(data.class), xfcc_array_len, 1);
      xfcc_description = repelem(string(data.label), xfcc_array_len, 1);
      xfcc_variable_names = [xfcc_mpu_ax_coeffNames, xfcc_mpu_ay_coeffNames, xfcc_mpu_az_coeffNames, ...
                             xfcc_mpu_gx_coeffNames, xfcc_mpu_gy_coeffNames, xfcc_mpu_gz_coeffNames];


      % Build xfcc features array
      xfcc_features = [xfcc_mpu_ax, xfcc_mpu_ay, xfcc_mpu_az, ...
                       xfcc_mpu_gx, xfcc_mpu_gy, xfcc_mpu_gz];

      % Build xfcc features table
      xfcc_features_tab = array2table(xfcc_features, 'VariableNames', xfcc_variable_names);
      xfcc_features_tab.labels = xfcc_labels;
      xfcc_features_tab.descr = xfcc_description;
      
      display_xfcc_mpu_data(xfcc_mpu_ax,xfcc_mpu_ay,xfcc_mpu_az, ...
                            xfcc_mpu_gx,xfcc_mpu_gy,xfcc_mpu_gz, ...
                            data.testee,data.id,data.class,data.label);
  
end


function xfcc_coeff_vector = XFCC_one_dataSeries(dataSeries, numCoeffs, originalFreq, ...
                                                    fs, bandEdges, WL, OL)
  % Useful info:
  % MPU frequency used = 50Hz
  % xfcc_coeff_vector is (numb_of_windows x numCoeffs) size
  % -------------------------------------------------------
  
  % resample data and normalize it
  s= resample(dataSeries, fs, originalFreq);
  s= s ./ 8; 
  
  % calculate XFCC
  m= mfcc(s,fs,'NumCoeffs',numCoeffs,'BandEdges',bandEdges,'WindowLength',WL,'OverlapLength',OL);
  xfcc_coeff_vector = m;
end


function xfcc_var_names = gen_table_variables_names(prefix, coeff_numb)
  % coeff_names for each axis data should contain following info:
  %
  %     {<prefix>_E}, {<prefix>_1}, {<prefix>_2}, ..., {<prefix>_<coeff_numb>}
  %
  % where first name is the Energy column of xfcc, and others are xfcc
  % coefficients
  xfcc_var_names = {sprintf('%s_E', prefix)};
  for i=1:coeff_numb
     xfcc_var_names = [xfcc_var_names, {sprintf('%s_c%d', prefix, i)}]; 
  end
end

function display_xfcc_mpu_data(mpu_ax,mpu_ay,mpu_az,mpu_gx,mpu_gy,mpu_gz, ...
                               testee, rec_id, class, info)
  f = figure;
  
  fig_name1 = sprintf("Linear Frequency Cepstral coefficients MPU6050: %s id-%d %s %s", testee, rec_id, class, info);
  display_xfcc_single_sensor(mpu_ax, mpu_ay, mpu_az, 'mpu__accl', fig_name1);
  display_xfcc_single_sensor(mpu_gx, mpu_gy, mpu_gz, 'mpu__gyro');
  
end

function display_xfcc_single_sensor(xfcc_axis_x, xfcc_axis_y, xfcc_axis_z, sensor, fig_name)
  % sensor - values from {'adxl', 'mpu_accl', 'mpu_gyro'}
  % all sensor data are displayed in a column
  
  idx = -1;
  switch sensor
      case 'mpu__accl'
          idx = 0;
      case 'mpu__gyro'
          idx = 1;
      otherwise
          fprintf("display_xfcc_single_sensor()# Error: wrong sensor name\n");
          return
  end
  
  lifter= 50;
  ceplifter=@(N,L)(1+0.5*L*sin(pi*[0:N-1]/L));
  
  % X-axis
  m_energy=xfcc_axis_x(:,1)'; m_coeffs=xfcc_axis_x(:,2:end)';
  m_coeffs_liftered= ceplifter(size(m_coeffs,1),lifter)' .* m_coeffs;
  
  subplot(6,3, [1,4]+9*idx);
  imagesc(m_coeffs_liftered); set(gca,'YDir','normal');
  ylabel(sprintf('X-axis %s',sensor)); set(gca,'XTick','');
  
  subplot(6,3, 7+9*idx); plot(m_energy);
  ylabel('Energy (a.u.)'); xlabel('Analysis window');
  
  % Y-axis
  m_energy=xfcc_axis_y(:,1)'; m_coeffs=xfcc_axis_y(:,2:end)';
  m_coeffs_liftered= ceplifter(size(m_coeffs,1),lifter)' .* m_coeffs;
  
  subplot(6,3, [2,5]+9*idx);
  imagesc(m_coeffs_liftered); set(gca,'YDir','normal');
  ylabel(sprintf('Y-axis %s',sensor)); set(gca,'XTick','');
  if (idx==0)
      title(fig_name);
  end
  
  subplot(6,3, 8+9*idx); plot(m_energy);
  ylabel('Energy (a.u.)'); xlabel('Analysis window');
  
  % z-axis
  m_energy=xfcc_axis_z(:,1)'; m_coeffs=xfcc_axis_z(:,2:end)';
  m_coeffs_liftered= ceplifter(size(m_coeffs,1),lifter)' .* m_coeffs;
  
  subplot(6,3, [3,6]+9*idx);
  imagesc(m_coeffs_liftered); set(gca,'YDir','normal');
  ylabel(sprintf('Z-axis %s',sensor)); set(gca,'XTick','');
  
  subplot(6,3, 9+9*idx); plot(m_energy);
  ylabel('Energy (a.u.)'); xlabel('Analysis window');
  
end
