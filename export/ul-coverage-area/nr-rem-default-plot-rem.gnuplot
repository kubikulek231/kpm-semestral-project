set xlabel "x-coordinate (m)"
set ylabel "y-coordinate (m)"
set cblabel "SNR (dB)"
set cblabel offset 3
unset key
set terminal png
set output "nr-rem-default-snr.png"
set size ratio -1
set cbrange [-5:30]
set xrange [-40:80]
set yrange [-70:50]
set xtics font "Helvetica,17"
set ytics font "Helvetica,17"
set cbtics font "Helvetica,17"
set xlabel font "Helvetica,17"
set ylabel font "Helvetica,17"
set cblabel font "Helvetica,17"
plot "nr-rem-default.out" using ($1):($2):($4) with image
set xlabel "x-coordinate (m)"
set ylabel "y-coordinate (m)"
set cblabel "SINR (dB)"
set cblabel offset 3
unset key
set terminal png
set output "nr-rem-default-sinr.png"
set size ratio -1
set cbrange [-5:30]
set xrange [-40:80]
set yrange [-70:50]
set xtics font "Helvetica,17"
set ytics font "Helvetica,17"
set cbtics font "Helvetica,17"
set xlabel font "Helvetica,17"
set ylabel font "Helvetica,17"
set cblabel font "Helvetica,17"
plot "nr-rem-default.out" using ($1):($2):($5) with image
set xlabel "x-coordinate (m)"
set ylabel "y-coordinate (m)"
set cblabel "IPSD (dBm)"
set cblabel offset 3
unset key
set terminal png
set output "nr-rem-default-ipsd.png"
set size ratio -1
set cbrange [-100:-20]
set xrange [-40:80]
set yrange [-70:50]
set xtics font "Helvetica,17"
set ytics font "Helvetica,17"
set cbtics font "Helvetica,17"
set xlabel font "Helvetica,17"
set ylabel font "Helvetica,17"
set cblabel font "Helvetica,17"
plot "nr-rem-default.out" using ($1):($2):($6) with image
set xlabel "x-coordinate (m)"
set ylabel "y-coordinate (m)"
set cblabel "SIR (dB)"
set cblabel offset 3
unset key
set terminal png
set output "nr-rem-default-sir.png"
set size ratio -1
set cbrange [-5:30]
set xrange [-40:80]
set yrange [-70:50]
set xtics font "Helvetica,17"
set ytics font "Helvetica,17"
set cbtics font "Helvetica,17"
set xlabel font "Helvetica,17"
set ylabel font "Helvetica,17"
set cblabel font "Helvetica,17"
plot "nr-rem-default.out" using ($1):($2):($7) with image
