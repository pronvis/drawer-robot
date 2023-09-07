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
