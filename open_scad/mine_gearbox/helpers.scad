module apply_split(height) {
    if (split) {
        g(Z(height)) children();
    } else {
        children();
    }
}

module nema_axle(diameter, height) {
    difference() {
        up_tube(d = diameter, h = height);
        // axle inside cut
        g(Z(-0.1)) intersection() {
            up_tube(d = diameter, h = height + 0.2);
            g(X(4), Z((height + 0.2) / 2)) 
                box(4, y = 4, h = height + 0.2);
        };
    }
}

module up_tube(d, h, solid = true) {
    g(TOUP()) tube(d=d, h=h, solid=solid);
}

module rounded_octagon(width,radius,height)
{
    r=radius;
    w=width-2*radius;
    a=w/(1+sqrt(2));
    
    p1=[-w/2,-a/2];
    p2=[-w/2,+a/2];
    p3=[-w/2+a/sqrt(2),+a/2+a/sqrt(2)];
    p4=[+w/2-a/sqrt(2),+a/2+a/sqrt(2)];
    p5=[+w/2,+a/2];
    p6=[+w/2,-a/2];
    p7=[+w/2-a/sqrt(2),-a/2-a/sqrt(2)];
    p8=[-w/2+a/sqrt(2),-a/2-a/sqrt(2)];
    
    points=[p1,p2,p3,p4,p5,p6,p7,p8];
    linear_extrude(height=height)
    translate([w/2,a/2,5])
    minkowski() 
    {
        polygon(points);
        translate([-w/2,-a/2,0])
        circle(r);
    }
}

module v_gear(
	mm_per_tooth    = 3,    //this is the "circular pitch", the circumference of the pitch circle divided by the number of teeth
	number_of_teeth = 11,   //total number of teeth around the entire perimeter
	thickness       = 6,    //thickness of gear in mm
	hole_diameter   = 3,    //diameter of the hole in the center, in mm
	twist           = 0,    //teeth rotate this many degrees from bottom of gear to top.  360 makes the gear a screw with each thread going around once
	teeth_to_hide   = 0,    //number of teeth to delete to make this only a fraction of a circle
	pressure_angle  = 28,   //Controls how straight or bulged the tooth sides are. In degrees.
	clearance       = 0.0,  //gap between top of a tooth on one gear and bottom of valley on a meshing gear (in millimeters)
	backlash        = 0.0,   //gap between two meshing teeth, in the direction along the circumference of the pitch circle
    center = false,   // center gear by z axis
    $fn = 20   // number of fragments to draw hole cylinder
) {
g(Z(thickness / 2))
    two()
        reflectZ(sides())
            gear(
                mm_per_tooth,
                number_of_teeth,
                thickness / 2,
                hole_diameter,
                twist,
                teeth_to_hide,
                pressure_angle,
                clearance,
                backlash,
                center,
                $fn = $fn
            );
}

module v_inner_gear (
	mm_per_tooth    = 3,    //this is the "circular pitch", the circumference of the pitch circle divided by the number of teeth
	number_of_teeth = 11,   //total number of teeth around the entire perimeter
	thickness       = 6,    //thickness of gear in mm
	outer_diameter  = 15,   //mm_per_tooth * (number_of_teeth + 3 ) / 3.1416 ,    //diameter of the hole in the center, in mm
	twist           = 0,    //teeth rotate this many degrees from bottom of gear to top.  360 makes the gear a screw with each thread going around once
	teeth_to_hide   = 0,    //number of teeth to delete to make this only a fraction of a circle
	pressure_angle  = 28,   //Controls how straight or bulged the tooth sides are. In degrees.
	clearance       = 0.0,  //gap between top of a tooth on one gear and bottom of valley on a meshing gear (in millimeters)
	backlash        = 0.0   //gap between two meshing teeth, in the direction along the circumference of the pitch circle
) {
    g(Z(thickness / 2))
        two()
            reflectZ(sides())
                inner_gear(
                    mm_per_tooth,
                    number_of_teeth,
                    thickness / 2,
                    outer_diameter,
                    twist,
                    teeth_to_hide,
                    pressure_angle,
                    clearance,
                    backlash
                );
};

