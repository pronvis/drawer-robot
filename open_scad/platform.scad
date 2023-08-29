include <constructive-compiled.scad>
$skinThick=5;

full_size = [100, 50, 50];
dymh_diameter = 6;
dymh_big_diameter = 14;
wall_width = 5;

partColors = [
    ["base", yellow, 0.8],
    ["screws",grey,0.8],
    ["screwMounts",red,.8],
    ["stm32",green,.8],
    ["top",blue,.8]
];

assemble("base,stm32,top,screws,screwMounts,stm32_protection") {
    base();
    top();
}


module base() autoColor(custom=partColors)
{
    add("base") chamfer(0, 0, -dymh_big_diameter, fnCorner=2) box(x = full_size[0], y = full_size[1], z = full_size[2]);
    remove("base") chamfer(0, 0, -dymh_big_diameter, fnCorner=2) box(x = full_size[0] - wall_width, y = full_size[1] - wall_width, z = full_size[2] - wall_width);
    remove("base") two() reflectY(sides()) two() reflectX(sides()) g(X(-full_size[0]/2), Y(-full_size[1]/2), turnXZ(90), turnYZ(45)) tube(d=dymh_diameter,h=40, solid=true);

    two() reflectY(sides()) two() reflectX(sides()) g(X(-full_size[0]/3 + 10), Y(-full_size[1]/2 + wall_width + 2), Z(-full_size[2]/4)) {
        hull() add("screwMounts") tube(d=8, h=full_size[2]/2, solid=true) Y(-wall_width/2) box(8, z=full_size[2]/2);
        remove("screwMounts,base") Z(-full_size[2]/4 + 5) screwM4(h=full_size[2]/2, noNut=true);
    }

    remove("base") align(TOUP) box(x = full_size[0]+10, y = full_size[1]+10, z = full_size[2]);
}

module top() autoColor(custom=partColors)
{
    g(Z(full_size[2])) {
        add("top") chamfer(0, 0, -dymh_big_diameter, fnCorner=2) box(x = full_size[0], y = full_size[1], z = full_size[2] / 4);
        add("stm32_protection") pieces(2) g(X(-full_size[0]/2 + full_size[0]/4 + 10), X(span(full_size[0]/2 - 20)), turnYZ(90)) tube(d=4, h=full_size[1], solid=true);
        remove("top") chamfer(0, 0, -dymh_big_diameter, fnCorner=2) box(x = full_size[0] - wall_width, y = full_size[1] - wall_width, z = full_size[2] / 2 + wall_width);
        remove("top") two() reflectY(sides()) two() reflectX(sides()) g(X(-full_size[0]/2), Y(-full_size[1]/2), Z(-full_size[2]/8), turnXZ(90), turnYZ(45)) tube(d=dymh_diameter,h=40, solid=true);

        mount_height = 3;
        two() reflectY(sides()) two() reflectX(sides()) g(X(-full_size[0]/3 + 10), Y(-full_size[1]/2 + wall_width + 2), Z(-full_size[2]/8 + mount_height/2)) {
            hull() add("screwMounts") tube(d=8, h=mount_height, solid=true) Y(-wall_width/2) box(8, h=mount_height);
            remove("screwMounts,top") Z(-mount_height) screwM4(h=full_size[2]/4, noNut=true);
        }
    }
}

between_pins = 2.54;
stm32_size = [53, 23, 25];
module stm32_volume() {
    difference() {
        cube(stm32_size); // размер который занимает stm32 
        for (pin = [0 : 19]) {
            translate([1.85 + between_pins * pin, 3, 0]) {
                cube([1, 1, stm32_size[2]]);
            }
        }

        for (pin = [0 : 19]) {
            translate([1.85 + between_pins * pin, stm32_size[1] - 3, 0]) {
                cube([1, 1, stm32_size[2]]);
            }
        }
    }
}
// stm32_volume();


// DYMH-106 size
// {
//     8 - height
//     13 diameter 
// }
function bottom_corners(sizes) = [[0, 0 ,0], [sizes[0], 0, 0], [sizes[0], sizes[1], 0], [0, sizes[1], 0]];

module triangle(katet_size) {
    CubePoints = [
        [  0,  0,  0 ],  //0
        [ katet_size,  0,  0 ],  //1
        [  0,  katet_size,  0 ],  //2
        [  0,  0,  full_size[2] ],  //3
        [ katet_size,  0,  full_size[2] ],  //4
        [  0,  katet_size,  full_size[2] ] //5
    ];

    CubeFaces = [
        [0,1,2],  // bottom
        [3,4,1,0],  // front
        [5,4,3],  // top
        [4,5,2,1],  // back
        [5,3,0,2]
    ]; // left

    polyhedron( CubePoints, CubeFaces );
}

module platform() {
    difference() {
        cube(full_size);
        corners = bottom_corners(full_size);

        for (i = [0:3]) {
            translate( [corners[i][0], corners[i][1], 0] ) {
                rotate([0, 0, 90 * i]) {
                    triangle(10); // for katet=10 gipotenuza will be ~14, which is close to DYMH-106 diameter (biggest part)
                }
            }
        }
    }
}
module inside_platform() {
    local_cube = [for (x = full_size) x - wall_width * 2];
    triangle_katet = 10;
    translate([wall_width, wall_width, wall_width]) {
        difference() {
            cube(local_cube);

            corners = bottom_corners(local_cube);
            for (i = [0:3]) {
                translate([corners[i][0], corners[i][1], 0]) {
                    rotate([0, 0, 90 * i]) {
                        triangle(triangle_katet);
                    }
                }
            }
        }
    }
} 

module plain_drawing() {
    difference() {
        platform();
        inside_platform();
        dymh_holes();
        translate([-10, -10, full_size[2] / 2]) {
            cube([500, 500, 200]);
        }
    }
}

module dymh_holes() {
    corners = bottom_corners(full_size);
    for (i = [0:3]) {
        translate([corners[i][0], corners[i][1], 0]) {
            translate([0, 0, full_size[2] / 2]) {
                rotate([90, 0 , 135 + 90 * i]) {
                    cylinder(20, d1 = dymh_diameter , d2 = dymh_diameter);
                }
            }
        }
    }
}
// plain_drawing();
// module pit_posts() {
//     cylinder()
// }
