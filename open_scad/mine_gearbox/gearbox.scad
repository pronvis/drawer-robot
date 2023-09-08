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
NEMA17_WIDTH = 42.3;
NEMA17_HOLES_SQUARE_WIDTH = 31;

//----------- MOTOR MOUNT ----------- 
MOTOR_BASE_HEIGHT = 9;                 // motor base for gearbox mounting height

//----------- MOTOR GEAR ----------- 
PLANET_GEAR_INNER_DIAMETER = 6.0;         // planet gear mountig hole inner diameter
SUN_GEAR_DIST_DIAMETER = 13.9;       // diameter of bottom sun gear distancer/support
MOTOR_MOUNT_INNER_DIAMETER = 5 + margin;     // motor gear mount inner diameter, cause nema17 axle diameter = 5
MOTOR_MOUNT_OUTPUT_DIAMETER = 20;  // motor gear mount outter diameter
MOTOR_GEAR_DIST_HEIGHT = 2; // motor gear distancer/support height
MOTOR_GEAR_DIST_DIAMETER = SUN_GEAR_DIST_DIAMETER; // motor gear distancer/support diameter
MOTOR_GEAR_HEIGHT = 5.3;    // motor gear height/tickness


//----------- GEARS ----------------
backlash = 0.05;
clearance = 0.2;
tooth_twist = 20;
h = 1; //wall
SUN_TOOTH_COUNT = 10;
SATELLITE_TOOTH_COUNT = 11;
OUTER_RING_TOOTH_COUNT = SUN_TOOTH_COUNT + 2 * SATELLITE_TOOTH_COUNT;
dd3 = (NEMA17_WIDTH - h * 2 - 0.4) * OUTER_RING_TOOTH_COUNT / (OUTER_RING_TOOTH_COUNT + 2 );
mm_per_tooth = dd3 / OUTER_RING_TOOTH_COUNT * PI; //all meshing gears need the same mm_per_tooth (and the same pressure_angle)
satellite_distance = pitch_radius(mm_per_tooth, SUN_TOOTH_COUNT) + pitch_radius(mm_per_tooth, SATELLITE_TOOTH_COUNT);
SATELLITES_COUNT = 3;

//------- CARRIER BOTTOM -----------
CARRIER_BOTTOM_MAIN_DIAMETER = 35;
//------- OUTER RING -----------

///////////////////////////////////
//////////// CALCULATED ///////////
///////////////////////////////////
h_motor_gear = MOTOR_BASE_HEIGHT + MOTOR_GEAR_DIST_HEIGHT + MOTOR_GEAR_HEIGHT;
ring_gear_inner_diameter = 20; //TODO: to calculate
//---------------------------------------------------------------
//------------- DISPLAY OPTIONS ---------------------------------
//---------------------------------------------------------------
split = 0;
split_free_space = 5;

all = "nema17,motor_mount,motor_gear,carrier_bottom,planets,ring_gear";
all_but_ring = "nema17,motor_mount,motor_gear,carrier_bottom,planets";
motor_mount = "motor_mount";
motor_gear = "motor_gear";
carrier_bottom = "carrier_bottom";
gear_w_planets = "motor_gear,planets";
planets = "planets";
ring_gear = "ring_gear";
gears = "motor_gear,ring_gear,planets";

assemble(ring_gear) {
    nema17();

    m_mount_height = MOTOR_BASE_HEIGHT;
    apply_split(NEMA17_AXLE_HEIGHT + split_free_space) {
        motor_mount(h = m_mount_height, hole_d = CARRIER_BOTTOM_MAIN_DIAMETER); 
    }

    apply_split(NEMA17_AXLE_HEIGHT + m_mount_height + split_free_space * 2)
        motor_gear(h = h_motor_gear, gear_h = MOTOR_GEAR_HEIGHT, gear_dist_h = MOTOR_GEAR_DIST_HEIGHT); 

    planets_height = MOTOR_GEAR_HEIGHT - 1;
    apply_split(NEMA17_AXLE_HEIGHT + h_motor_gear - MOTOR_GEAR_DIST_HEIGHT + split_free_space * 3)
        Z(m_mount_height + MOTOR_GEAR_DIST_HEIGHT) {
            planets(h = planets_height);
        }

    apply_split(NEMA17_AXLE_HEIGHT +  h_motor_gear + planets_height + split_free_space * 4)
        Z(m_mount_height) {
            height = MOTOR_GEAR_DIST_HEIGHT * 2 + MOTOR_GEAR_HEIGHT;
            ring_gear(h = height, hole_d = CARRIER_BOTTOM_MAIN_DIAMETER, inside_gear_h = planets_height);
        }

    //TODO
    // apply_split(NEMA17_AXLE_HEIGHT + m_mount_height + h_motor_gear + split_free_space * 3)
    //     carrier_bottom();
}

module nema17() autoColor(custom = partColors) {
    g(X(-4))
        add("nema17") import("Nema17.stl");
}

module motor_mount(h, hole_d) autoColor(custom = partColors) {
    add("motor_mount") {
        base(h = h, hole_d = hole_d);
    }
}

module motor_gear(h, gear_h, gear_dist_h) autoColor(custom = partColors) {
    add("motor_gear", remove = "carrier_bottom") {
        //base
        //TODO: think how to use this feature
        base_h = h - gear_dist_h - gear_h;
        Z($removing? -0.1 : 0 ) up_tube(d = margin(MOTOR_MOUNT_OUTPUT_DIAMETER, 1), h = margin( base_h , 0.2 ));

        //gear distancer
        gear_distancer_h = h - base_h - gear_h;
        Z(base_h) up_tube(d = MOTOR_GEAR_DIST_DIAMETER, h = gear_distancer_h);

        //filling cylinder
        Z(base_h + gear_distancer_h) up_tube(d = PLANET_GEAR_INNER_DIAMETER, h = gear_h);

        //gear
        Z(base_h + gear_distancer_h) {
            v_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = SUN_TOOTH_COUNT, thickness = gear_h, hole_diameter = PLANET_GEAR_INNER_DIAMETER - 0.5, clearance = clearance, backlash = backlash, twist = tooth_twist+25);
        }
    }

    //hole for nema axle
    remove("motor_gear") {
        g(Z(-0.1)) {
            nema_axle(MOTOR_MOUNT_INNER_DIAMETER, h + 0.2);
        }
    }
}

module carrier_bottom() autoColor(custom = partColors) {
    add("carrier_bottom") {
        up_tube(d = CARRIER_BOTTOM_MAIN_DIAMETER - 1, h = MOTOR_BASE_HEIGHT);
    }

//        g(X(10),turnXY(45),solid())
//   {
//     add("rod",remove="plate")
//       tube(d=margin(16,1),h=20);

//     add("plate")
//       box(margin(20),h=3);
//    }


    // remove("carrier_bottom") {
    //     Z(-0.1)
    //         up_tube(d = MOTOR_MOUNT_OUTPUT_DIAMETER+1, h = MOTOR_BASE_HEIGHT + 0.2);
    // }
}

module planets(h) autoColor(custom = partColors) {
    add("planets") {
        pieces(SATELLITES_COUNT) g(turnXY(spanAllButLast(360)), X(satellite_distance)) {
            g(turnXZ(180), Z(-MOTOR_GEAR_HEIGHT/2 - h/2)) 
                v_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = SATELLITE_TOOTH_COUNT, thickness = h, hole_diameter = 4, clearance = clearance, backlash = backlash, twist = tooth_twist+25);
        };
    }
}

module ring_gear(h, hole_d, inside_gear_h) autoColor(custom = partColors) {
    // OLD SOLUTION
    inside_gear_height = inside_gear_h + 1;
    add("ring_gear") {
        intersection() {
            base(h = h, hole_d = hole_d);
            g(turnXZ(180), Z(-inside_gear_height/2 - h/2)) 
                v_inner_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = OUTER_RING_TOOTH_COUNT, outer_diameter = NEMA17_WIDTH, thickness = inside_gear_height, clearance = clearance, backlash = backlash, twist = tooth_twist);
        }
    }

    // NEW SOLUTION
    // add("ring_gear") {
    //     base(h = h, hole_d = hole_d);
    // }

    // remove("ring_gear") {
    // fake_gears_count = 8;
    // fake_gear_h = inside_gear_h + 1;
    // pieces(fake_gears_count) turnXY(360 / fake_gears_count) 
    //         g(turnXY(spanAllButLast(360)), X(satellite_distance), turnXZ(180), Z(-MOTOR_GEAR_HEIGHT/2 - fake_gear_h/2 - MOTOR_GEAR_DIST_HEIGHT)) 
    //             v_gear(mm_per_tooth = mm_per_tooth, number_of_teeth = SATELLITE_TOOTH_COUNT, thickness = fake_gear_h, hole_diameter = 4, clearance = clearance, backlash = backlash, twist = tooth_twist+25);
    // }
}
