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
full_size = [200, 100, 50];
function bottom_corners(sizes) = [[0, 0 ,0], [sizes[0], 0, 0], [sizes[0], sizes[1], 0], [0, sizes[1], 0]];

dymh_diameter = 6;

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

wall_width = 5;
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

difference() {
    platform();
    inside_platform();
    dymh_holes();
    translate([-10, -10, full_size[2] / 2]) {
        cube([500, 500, 200]);
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

// module pit_posts() {
//     cylinder()
// }
