$fn= $preview ? 32 : 64;

total_height = 34;

base_height = 6;

center_height = total_height - base_height*2;
center_radius = 10;

base_radius = center_radius + 3;

base_shift = center_height / 2 + base_height/4;

module hole() {
    difference() {
        cylinder(total_height, 2.5, 2.5, center=true);
        translate([4, 0, 0]) {
            cube([4, 4, total_height], center=true);
        }   
    }
}

module center() {
    difference() {
        cylinder(center_height, center_radius, center_radius, center = true);
        hole();
    }
}

module base() {
    difference(){
        union() {        
            cylinder(base_height/2, base_radius, center_radius, center = true);
            translate([0, 0, -base_height/2]) {
                cylinder(base_height/2, base_radius, base_radius, center = true);
            }
        }
        hole();
    }
}

center();

translate([0, 0, -base_shift]) {
   base();
}

translate([0, 0, base_shift]) {
    rotate([180,0,0]) { 
        base();
    }
}
