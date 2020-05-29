To show the cepstral coefficients during chosen activity run in Matlab:

load measurements_data.mat;
dat1 = measurements_data{1,1};
show_cepstrum_features_MPU(dat1, 10, 40, 3.0, 2.0, 0.5, 30);

-------------------------------------------------------------

To run the classification on features data write in Matlab:

SVM = load('SVM_FineGaussian_optimized.mat').SVM_FineGaussian_optimized;
x = load('features_tab.mat').features_tab;
y = SVM.predictFcn(x);