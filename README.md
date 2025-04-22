# Variable-camber-generator

#### This code generates coordinates for a 4-digit NACA airfoil with a variable-camber morphing trailing edge, leading edge or both.

LE_console.cpp generates morphing leading edge (slat)<br />
TE_console.cpp generates morphing trailing edge (flap)<br />
TE_trimmed_console.cpp generates trimmed morphing trailing edge<br />
ALL_console.cpp generates the complete set of high-lift devices<br />

##### HOW IT WORKS (Trailing Edge Flap Example):

The user defines the hinge x-location (x<sub>h</sub>), with y<sub>h</sub> calculated as the mean camber line point at x<sub>h</sub>. Upper (x<sub>u</sub>, y<sub>u</sub>) and lower (x<sub>l</sub>, y<sub>l</sub>) surface coordinates of the morphing section are rotated using:

x<sub>u new</sub> = x<sub>h</sub> + (x<sub>u</sub> - x<sub>h</sub>) cos (δ<sub>ld</sub>) - (y<sub>u</sub> - y<sub>h</sub>) sin (δ<sub>ld</sub>)
y<sub>u new</sub> = y<sub>h</sub> + (x<sub>u</sub> - x<sub>h</sub>) sin (δ<sub>ld</sub>) + (y<sub>u</sub> - y<sub>h</sub>) cos (δ<sub>ld</sub>)
x<sub>l new</sub> = x<sub>h</sub> + (x<sub>l</sub> - x<sub>h</sub>) cos (δ<sub>ld</sub>) - (y<sub>l</sub> - y<sub>h</sub>) * sin (δ<sub>ld</sub>)
y<sub>l new</sub> = y<sub>h</sub> + (x<sub>l</sub> - x<sub>h</sub>) sin (δ<sub>ld</sub>) + (y<sub>l</sub> - y<sub>h</sub>) cos (δ<sub>ld</sub>)

δ<sub>ld</sub> is the local deflection angle, calculated for each point x in the morphing section:

δ<sub>ld</sub> = δ<sub>fd</sub>  (x - x<sub>h</sub>) / (1 - x<sub>h</sub>), where δ<sub>аd</sub> is the full deflection angle.

Each point on the airfoil surface at x > x<sub>h</sub> deflects by an angle relative to its previous position (δ<sub>ld</sub>). The code accounts for the mean camber line slope at (x<sub>h</sub>, y<sub>h</sub>):

δ<sub>fd</sub> = δ<sub>d</sub> + δ<sub>h</sub>, 

where d<sub>d</sub> is the user-defined deflection angle and d<sub>h</sub> is the tangent angle to the mean camber line at (x<sub>h</sub>, y<sub>h</sub>).

Similar algorithms apply to leading edge (slat). For the trimmed flap, the local deflection angle δ<sub>ld te</sub> is:

δ<sub>ld te</sub> = -δ<sub>fd</sub>  (x<sub>tr</sub> - x<sub>h</sub>) / (1 - x<sub>h</sub>) (x - x<sub>tr</sub>) / (1 - x<sub>tr</sub>)
