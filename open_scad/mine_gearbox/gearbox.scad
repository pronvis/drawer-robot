include <constructive-compiled.scad>
include <helpers.scad>
//for rendering
// $fn=100;
$fn=40;

// COLORS
white = [1, 1, 1];
dark = [0.25, 0.25, 0.25];
partColors = [
    ["nema17", white, 1],
    ["motor_mount", dark, 1],
    ["motor_gear", [0, 0.3, 0.5], 1],
];

///////////////////////////////////
//////////// CONSTANTS ////////////
///////////////////////////////////
NEMA17_AXLE_HEIGHT = 22;

//----------- MOTOR MOUNT ----------- 
GEARBOX_OUTER_DIAMETER = 42;           // gearbox outter diameter/width
MOTOR_BASE_HEIGHT = 9;                 // motor base for gearbox mounting height
MOUNTING_HOLES_COUNT = 4;
MOUNTING_HOLE_OUTER_DIAMETER = 10;     // gearbox mounting holes outter diameter
MOUNTING_HOLE_POSITION = 22;           // position of the gearbox mounting holes
MOUNTING_HOLE_INNER_DIAMETER = 3.3;    // gearbox mounting holes inner diameter

//----------- MOTOR GEAR ----------- 
PLANET_GEAR_INNER_DIAMETER = 6.0;         // planet gear mountig hole inner diameter
SUN_GEAR_HEIGHT = 5.3;             // sun gear height
SUN_GEAR_DIST_HEIGHT = 2.1;        // height of bottom sun gear distancer/support
SUN_GEAR_DIST_DIAMETER = 13.9;       // diameter of bottom sun gear distancer/support
MOTOR_MOUNT_INNER_DIAMETER=5.25;     // motor gear mount inner diameter, cause nema17 axle diameter = 5
MOTOR_MOUNT_HEIGHT = 7.5;          // motor gear mount height 
MOTOR_MOUNT_OUTPUT_DIAMETER = 20;  // motor gear mount outter diameter
MOTOR_GEAR_DIST_HEIGHT = SUN_GEAR_DIST_HEIGHT; // motor gear distancer/support height
MOTOR_GEAR_DIST_DIAMETER = SUN_GEAR_DIST_DIAMETER; // motor gear distancer/support diameter
MOTOR_GEAR_HEIGHT = SUN_GEAR_HEIGHT;    // motor gear height/tickness
// MOTOR_MOUNT_INNER_DIAMETER = 5.25;      // motor gear mount inner diameter
///////////////////////////////////
//////////// CALCULATED ///////////
///////////////////////////////////

ring_gear_inner_diameter = 20; //TODO: to calculate

//---------------------------------------------------------------
//------------- DISPLAY OPTIONS ---------------------------------
//---------------------------------------------------------------
split = 1;
split_free_space = 10;

all = "nema17,motor_mount,motor_gear";
mount = "motor_mount";
gear = "motor_gear";

assemble(gear) {
    nema17();

    apply_split(NEMA17_AXLE_HEIGHT + split_free_space)
        motor_mount(); 
    apply_split(NEMA17_AXLE_HEIGHT + MOTOR_BASE_HEIGHT + split_free_space * 2)
        motor_gear(); 
}

module nema17() autoColor(custom = partColors) {
        g(X(-4))
        add("nema17") import("Nema17.stl");
}

module motor_mount() autoColor(custom = partColors) {
    g(turnXY(45)) { 
        add("motor_mount") {
            rounded_octagon(width = GEARBOX_OUTER_DIAMETER, radius = 6, height = MOTOR_BASE_HEIGHT);

            pieces(MOUNTING_HOLES_COUNT) g(turnXY(spanAllButLast(360)), X(MOUNTING_HOLE_POSITION))
                up_tube(d = MOUNTING_HOLE_OUTER_DIAMETER, h = MOTOR_BASE_HEIGHT);
        };

        remove("motor_mount") {
            g(Z(-0.1)) {
                //central hole
                up_tube(d = ring_gear_inner_diameter, h = MOTOR_BASE_HEIGHT + 0.2);

                //mounting holes
                pieces(MOUNTING_HOLES_COUNT) g(turnXY(spanAllButLast(360)), X(MOUNTING_HOLE_POSITION))
                    up_tube(d = MOUNTING_HOLE_INNER_DIAMETER, h = MOTOR_BASE_HEIGHT + 0.2);
            }
        }
    }
}

module motor_gear() autoColor(custom = partColors) {
    add("motor_gear") {
        //base
        up_tube(d = MOTOR_MOUNT_OUTPUT_DIAMETER, h = MOTOR_MOUNT_HEIGHT);
        //gear distancer
        g(Z(MOTOR_MOUNT_HEIGHT)) up_tube(d = MOTOR_GEAR_DIST_DIAMETER, h = MOTOR_GEAR_DIST_HEIGHT);
        //filling cylinder
        g(Z(MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT)) up_tube(d = PLANET_GEAR_INNER_DIAMETER, h = MOTOR_GEAR_HEIGHT);
    }

    h_motor_gear = MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT + MOTOR_GEAR_HEIGHT + 0.2;
    remove("motor_gear") {
        g(Z(-0.1)) {
            nema_axle(MOTOR_MOUNT_INNER_DIAMETER, h_motor_gear);
        }
    }
}
