include <constructive-compiled.scad>
include <helpers.scad>
use <pd-gears.scad>

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
    ["carrier_bottom", [0.2, 0.2, 0.4], 1],
    ["planets", [0, 0.3, 0], 1],
    ["ring_gear", [0.2, 0.3, 0], 1],
];

///////////////////////////////////
//////////// CONSTANTS ////////////
///////////////////////////////////
margin = 0.25;
NEMA17_AXLE_HEIGHT = 22;

//----------- MOTOR MOUNT ----------- 
GEARBOX_OUTER_DIAMETER = 42;           // gearbox outter diameter/width
MOTOR_BASE_HEIGHT = 9;                 // motor base for gearbox mounting height
MOUNTING_HOLE_POSITION = 22;           // position of the gearbox mounting holes
MOUNTING_HOLE_INNER_DIAMETER = 3.3;    // gearbox mounting holes inner diameter

//----------- MOTOR GEAR ----------- 
PLANET_GEAR_INNER_DIAMETER = 6.0;         // planet gear mountig hole inner diameter
SUN_GEAR_HEIGHT = 5.3;             // sun gear height
SUN_GEAR_DIST_HEIGHT = 2.1;        // height of bottom sun gear distancer/support
SUN_GEAR_DIST_DIAMETER = 13.9;       // diameter of bottom sun gear distancer/support
MOTOR_MOUNT_INNER_DIAMETER = 5 + margin;     // motor gear mount inner diameter, cause nema17 axle diameter = 5
MOTOR_MOUNT_HEIGHT = 7.5;          // motor gear mount height 
MOTOR_MOUNT_OUTPUT_DIAMETER = 20;  // motor gear mount outter diameter
MOTOR_GEAR_DIST_HEIGHT = SUN_GEAR_DIST_HEIGHT; // motor gear distancer/support height
MOTOR_GEAR_DIST_DIAMETER = SUN_GEAR_DIST_DIAMETER; // motor gear distancer/support diameter
MOTOR_GEAR_HEIGHT = SUN_GEAR_HEIGHT;    // motor gear height/tickness

//----------- GEARS ----------------
backlash = 0.05;
clearance = 0.2;
tooth_twist = 20;
h = 1; //wall
SUN_TOOTH_COUNT = 10;
SATELLITE_TOOTH_COUNT = 11;
OUTER_RING_DIAMETER = GEARBOX_OUTER_DIAMETER;
OUTER_RING_TOOTH_COUNT = SUN_TOOTH_COUNT + 2 * SATELLITE_TOOTH_COUNT;
dd3 = (OUTER_RING_DIAMETER - h * 2 - 0.4) * OUTER_RING_TOOTH_COUNT / (OUTER_RING_TOOTH_COUNT + 2 );
mm_per_tooth = dd3 / OUTER_RING_TOOTH_COUNT * PI; //all meshing gears need the same mm_per_tooth (and the same pressure_angle)
satellite_distance = pitch_radius(mm_per_tooth, SUN_TOOTH_COUNT) + pitch_radius(mm_per_tooth, SATELLITE_TOOTH_COUNT);
SATELLITES_COUNT = 3;

//------- CARRIER BOTTOM -----------
CARRIER_BOTTOM_MAIN_DIAMETER = 35;
carrier_bottom_height = 10; //TODO: change me
//------- OUTER RING -----------

///////////////////////////////////
//////////// CALCULATED ///////////
///////////////////////////////////
h_motor_gear = MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT + MOTOR_GEAR_HEIGHT;
ring_gear_inner_diameter = 20; //TODO: to calculate
planets_height = 20; //TODO: to calculate
//---------------------------------------------------------------
//------------- DISPLAY OPTIONS ---------------------------------
//---------------------------------------------------------------
split = 0;
split_free_space = 10;

all = "nema17,motor_mount,motor_gear,carrier_bottom,planets,ring_gear";
all_but_ring = "nema17,motor_mount,motor_gear,carrier_bottom,planets";
motor_mount = "motor_mount";
motor_gear = "motor_gear";
carrier_bottom = "carrier_bottom";
gear_w_planets = "motor_gear,planets";
planets = "planets";
ring_gear = "ring_gear";
gears = "motor_gear,ring_gear,planets";

assemble(all_but_ring) {
    nema17();

    apply_split(NEMA17_AXLE_HEIGHT + split_free_space)
        motor_mount(); 

    apply_split(NEMA17_AXLE_HEIGHT + MOTOR_BASE_HEIGHT + split_free_space * 2)
        motor_gear(); 

    apply_split(NEMA17_AXLE_HEIGHT + MOTOR_BASE_HEIGHT + h_motor_gear + split_free_space * 3)
        carrier_bottom();

    apply_split(NEMA17_AXLE_HEIGHT + MOTOR_BASE_HEIGHT + h_motor_gear + carrier_bottom_height + split_free_space * 4)
        planets();

    apply_split(NEMA17_AXLE_HEIGHT + MOTOR_BASE_HEIGHT + h_motor_gear + carrier_bottom_height + planets_height + split_free_space * 5)
        ring_gear();
}

module nema17() autoColor(custom = partColors) {
    g(X(-4))
        add("nema17") import("Nema17.stl");
}

module motor_mount() autoColor(custom = partColors) {
    mounting_holes_count = 4;
    mounting_hole_outer_diameter = 10;     // gearbox mounting holes outter diameter

    g(turnXY(45)) { 
        add("motor_mount") {
            rounded_octagon(width = GEARBOX_OUTER_DIAMETER, radius = 6, height = MOTOR_BASE_HEIGHT);

            pieces(mounting_holes_count) g(turnXY(spanAllButLast(360)), X(MOUNTING_HOLE_POSITION))
                up_tube(d = mounting_hole_outer_diameter, h = MOTOR_BASE_HEIGHT);
        };

        remove("motor_mount") {
            g(Z(-0.1)) {
                //central hole
                up_tube(d = CARRIER_BOTTOM_MAIN_DIAMETER + 1, h = MOTOR_BASE_HEIGHT + 0.2);

                //mounting holes
                pieces(mounting_holes_count) g(turnXY(spanAllButLast(360)), X(MOUNTING_HOLE_POSITION))
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
        Z(MOTOR_MOUNT_HEIGHT) up_tube(d = MOTOR_GEAR_DIST_DIAMETER, h = MOTOR_GEAR_DIST_HEIGHT);

        //filling cylinder
        Z(MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT) up_tube(d = PLANET_GEAR_INNER_DIAMETER, h = MOTOR_GEAR_HEIGHT);

        //gear
        Z(MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT) {
            v_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = SUN_TOOTH_COUNT, thickness = MOTOR_GEAR_HEIGHT, hole_diameter = PLANET_GEAR_INNER_DIAMETER - 0.5, clearance = clearance, backlash = backlash, twist = tooth_twist+25);
        }
    }

    //hole for nema axle
    remove("motor_gear") {
        g(Z(-0.1)) {
            nema_axle(MOTOR_MOUNT_INNER_DIAMETER, h_motor_gear + 0.2);
        }
    }
}

module carrier_bottom() autoColor(custom = partColors) {
    add("carrier_bottom") {
        up_tube(d = CARRIER_BOTTOM_MAIN_DIAMETER - 1, h = MOTOR_MOUNT_HEIGHT);
    }

    remove("carrier_bottom") {
        Z(-0.1)
            up_tube(d = MOTOR_MOUNT_OUTPUT_DIAMETER+1, h = MOTOR_MOUNT_HEIGHT + 0.2);
    }
}

module planets() autoColor(custom = partColors) {
    add("planets") {
        g(Z(MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT)) {
            pieces(SATELLITES_COUNT) g(turnXY(spanAllButLast(360)), X(satellite_distance)) {
                g(turnXZ(180), Z(-MOTOR_GEAR_HEIGHT)) 
                    v_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = SATELLITE_TOOTH_COUNT, thickness = MOTOR_GEAR_HEIGHT, hole_diameter = 4, clearance = clearance, backlash = backlash, twist = tooth_twist+25);
            };
        }
    }
}

module ring_gear() autoColor(custom = partColors) {
    add("ring_gear") {
        g(Z(MOTOR_MOUNT_HEIGHT + MOTOR_GEAR_DIST_HEIGHT)) {
            g(turnXZ(180), Z(-MOTOR_GEAR_HEIGHT)) 
                v_inner_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = OUTER_RING_TOOTH_COUNT, outer_diameter = OUTER_RING_DIAMETER, thickness = MOTOR_GEAR_HEIGHT, clearance = clearance, backlash = backlash, twist = tooth_twist);
        }
    }
}
