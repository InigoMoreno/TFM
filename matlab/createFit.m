

%% Fit: 'untitled fit 1'.
[xData, yData] = prepareCurveData( us, K1 );

% Set up fittype and options.
ft = fittype( 'rat11' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.831359756387049 0.034833820567729 0.757838710385933];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, xData, yData, 'predobs' );
legend( h, 'K1 vs. us', 'untitled fit 1', 'Lower bounds (untitled fit 1)', 'Upper bounds (untitled fit 1)', 'Location', 'NorthEast' );
% Label axes
xlabel us
ylabel K1
grid on


