include <constructive-compiled.scad>
include <BOSL/constants.scad>
use <BOSL/involute_gears.scad>

GEAR_MODUL=1.25;            // gear module
PLANET_GEAR_N=11;            // planet gear tooth count
PLANET_GEAR_H=4.7;          // planet gear height
PLANET_GEAR_ID=6.0;         // planet gear mountig hole inner diameter
GEAR_PREASURE_ANGLE=20;     // gear preasure angle
GEAR_HELIX_ANGLE=0;         // gear helix angle

// General Variables
pi = 3.14159;
rad = 57.29578;
clearance = 0.1;   // clearance between teeth

/*  Converts Radians to Degrees */
function grad(pressure_angle) = pressure_angle*rad;
/*  Converts Degrees to Radians */
function radian(pressure_angle) = pressure_angle/rad;

/*  Spur gear
    
    diameter = Pitch Circle Diameter
    modul = Height of the Tooth Tip beyond the Pitch Circle
    tooth_number = Number of Gear Teeth
    width = tooth_width
    bore = Diameter of the Center Hole
    pressure_angle = Pressure Angle, Standard = 20° according to DIN 867. Should not exceed 45°.
    helix_angle = Helix Angle to the Axis of Rotation; 0° = Spur Teeth
    optimized = Create holes for Material-/Weight-Saving or Surface Enhancements where Geometry allows */
module spur_gear(diameter, modul, tooth_number, width, bore, pressure_angle = 20, helix_angle = 0, optimized = true)
{

    d = diameter;
    r = d / 2;                                                      // Pitch Circle Radius
    alpha_spur = atan(tan(pressure_angle)/cos(helix_angle));// Helix Angle in Transverse Section
    db = d * cos(alpha_spur);                                      // Base Circle Diameter
    rb = db / 2;                                                    // Base Circle Radius
    da = (modul <1)? d + modul * 2.2 : d + modul * 2;               // Tip Diameter according to DIN 58400 or DIN 867
    ra = da / 2;                                                    // Tip Circle Radius
    c =  (tooth_number <3)? 0 : modul/6;                                // Tip Clearance
    df = d - 2 * (modul + c);                                       // Root Circle Diameter
    rf = df / 2;                                                    // Root Radius
    rho_ra = acos(rb/ra);                                           // Maximum Rolling Angle;                                                              // Involute begins on the Base Circle and ends at the Tip Circle
    rho_r = acos(rb/r);                                             // Rolling Angle at Pitch Circle;
                                                                    // Involute begins on the Base Circle and ends at the Tip Circle
    phi_r = grad(tan(rho_r)-radian(rho_r));                         // Angle to Point of Involute on Pitch Circle
    gamma = rad*width/(r*tan(90-helix_angle));               // Torsion Angle for Extrusion
    step = rho_ra/16;                                            // Involute is divided into 16 pieces
    tau = 360/tooth_number;                                             // Pitch Angle
    
    r_hole = (2*rf - bore)/8;                                    // Radius of Holes for Material-/Weight-Saving
    rm = bore/2+2*r_hole;                                        // Distance of the Axes of the Holes from the Main Axis
    z_hole = floor(2*pi*rm/(3*r_hole));                             // Number of Holes for Material-/Weight-Saving

        cube(diameter);
}

// assemble() {
    // gear(mm_per_tooth = 8,number_of_teeth=11, thickness=8, hole_diameter=5, twist = 20);
     gear_tooth_profile(mm_per_tooth=5, number_of_teeth=20, pressure_angle=20);
    // gear(mm_per_tooth=5, number_of_teeth=20, thickness=10*cos(45), hole_diameter=5, twist=-30, bevelang=45, slices=12, $fa=1, $fs=1);
// }
