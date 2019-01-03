# Gnuplot for RMSE Error

set title "Root Mean Squared Error for Lidar and Radar Measurements" font "Helvetica,15"
set autoscale
unset log
unset label
set key at 190.0,4.75
set key font "Helvetica,12"
set xr [0:200]
set yr [0:5]
set xlabel "Iteration (count)" font "Helvetica,12"
set pointsize 0.75
set ylabel "RMSE" font "Helvetica,12"

plot "RMSE_Out.txt" using 1:2 title 'Position - X' with linespoints, \
     "RMSE_Out.txt" using 1:3 title 'Position - Y' with linespoints, \
     "RMSE_Out.txt" using 1:4 title 'Velocity - X' with linespoints, \
     "RMSE_Out.txt" using 1:5 title 'Velocity - Y' with linespoints
