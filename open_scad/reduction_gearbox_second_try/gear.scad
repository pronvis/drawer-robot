// Planetary gear bearing (customizable)

use <pins.scad>;
use <Write.scad>
//----------------------------------------------------------------------------
//PARAMETERS------------------------------------------------------------------
//----------------------------------------------------------------------------

object=1; //[0=planetary gear:1=carrier:2=both]

//Planetary gear--------------------------------------------------------------

approximate_gear_ratio=6; 
//The actual acheived gear ratio is influenced by the number_of_teeth_on_planets and number_of_planets
number_of_teeth_on_planets=22; 
number_of_planets=3;

// outer diameter of ring
D=120;
// thickness
T=20;
// clearance b
tol=0.15;

// pressure angle between gears
P=30;//[30:60] 
// number of teeth to twist across
nTwist=1;
// width of hexagonal hole
w=6.2;
// maximum depth ratio of teeth
DR=0.5;

//Planetary gear bearings-------------------------------------------------------

number_of_planets_b=4;
number_of_teeth_on_planets_b=7;
approximate_number_of_teeth_on_sun_b=8;
wall_thickness=4;
// pressure angle between gears
P_b=40;//[30:60] 
// number of teeth to twist across
nTwist_b=1;
w_b=4.2;// width of gear bearing hexagonal hole
DR_b=0.5;// maximum depth ratio of teeth

pin_diameter=-1; //[0=no pin:-1=auto. max:else=diameter]
pin_lip_height=3;
pin_lip_thickness=1;
pin_tolerance=0.2;

//Carrier-------------------------------------------------------
carrier_thickness=5;
carrier_gear_gap=2;
carrier_arm_width=18;
carrier_outer_fillet_radius=5;
carrier_inner_fillet_radius=20;
carrier_extra_length=10;
carrier_cylinder_radius=10;
carrier_cylinder_height=5;
w_c=6.2; //width of carrier hexagonal hole

//----------------------------------------------------------------------------
//DERIVED VALUES--------------------------------------------------------------
//----------------------------------------------------------------------------

//Planetary gear--------------------------------------------------------------
approximate_number_of_teeth_on_sun=(2*number_of_teeth_on_planets)/(approximate_gear_ratio-2);
m=round(number_of_planets);
np=round(number_of_teeth_on_planets);
ns1=approximate_number_of_teeth_on_sun;
k1=round(2/m*(ns1+np));
k= k1*m%2!=0 ? k1+1 : k1;
ns=k*m/2-np;
nr=ns+2*np;
pitchD=0.9*D/(1+min(PI/(2*nr*tan(P)),PI*DR/nr));
pitch=pitchD*PI/nr;
helix_angle=atan(2*nTwist*pitch/T);
gear_ratio=(nr+ns)/ns;
echo("gear_ratio=",gear_ratio);
echo("Number of teeth on planets",np);
echo("Number of teeth on sun",ns);
echo("Number of teeth on annular",nr);
echo("pitch",pitch);
echo("helix_angle",helix_angle);

phi=$t*360/m;

//Gear bearing--------------------------------------------------------------
Gear_planet_pitch_radius=np*pitch/(2*PI);
Gear_planet_inner_radius=Gear_planet_pitch_radius-DR/2-tol;
echo("Gear planet inner radius=",Gear_planet_inner_radius);
D_b=2*Gear_planet_inner_radius-2*wall_thickness;

m_b=round(number_of_planets_b);
np_b=round(number_of_teeth_on_planets_b);
ns1_b=approximate_number_of_teeth_on_sun_b;
k1_b=round(2/m_b*(ns1_b+np_b));
k_b= k1_b*m_b%2!=0 ? k1_b+1 : k1_b;
ns_b=k_b*m_b/2-np_b;
echo(ns_b);
nr_b=ns_b+2*np_b;
pitchD_b=D_b/(1+min(PI/(2*nr*tan(P_b)),PI*DR_b/nr_b));
pitch_b=pitchD_b*PI/nr_b;
echo(pitch_b);
helix_angle_b=atan(2*nTwist_b*pitch_b/T);
echo(helix_angle_b);

depth_val=pitch_b/(2*tan(P_b));
echo("depth_val",depth_val);
Bearing_sun_pitch_radius=ns_b*pitch_b/(2*PI);
echo("Bearing_sun_pitch_radius",Bearing_sun_pitch_radius);
Bearing_sun_root_radius = Bearing_sun_pitch_radius-depth_val/2-tol/2;
echo("Bearing_sun_root_radius",Bearing_sun_root_radius);

//carrier------------------------------------------------------------------
cl=pitchD/2*(ns+np)/nr+carrier_extra_length-carrier_outer_fillet_radius;
cw=carrier_arm_width-2*carrier_outer_fillet_radius;
carrier_translate_distance=(T+carrier_gear_gap)*(object-1);


//----------------------------------------------------------------------------
//MODEL-----------------------------------------------------------------------
//----------------------------------------------------------------------------

//Planetary gear--------------------------------------------------------------
if (object==0 || object==2){
	//Annuler 
	translate([0,0,T/2]){
		difference(){
			cylinder(r=D/2,h=T,center=true,$fn=100);
			union(){
				writecylinder(str(gear_ratio,":1"),[0,0,0],D/2,T,h=T/2,rotate=0,center=true); 
				herringbone(nr,pitch,P,DR,-tol,helix_angle,T+0.2);
			}
		}
	//sun
	rotate([0,0,(np+1)*180/ns+phi*(ns+np)*2/ns]){
		difference(){
			mirror([0,1,0])
				herringbone(ns,pitch,P,DR,tol,helix_angle,T);
			cylinder(r=w/sqrt(3),h=T+1,center=true,$fn=6);
		}
	}
//planets (gear bearings)-----------------------------------------------------
	for(i=[1:m])rotate([0,0,i*360/m+phi])translate([pitchD/2*(ns+np)/nr,0,0]){
	//Carrier mounting pin
		translate([0,0,T/2]){
			
			if (pin_diameter==0)
			{
				
			}
			else{
				//auto. max
				if (pin_diameter==-1)//
				{
					pin(h=carrier_thickness+carrier_gear_gap, r=Bearing_sun_root_radius, lh=pin_lip_height, lt=pin_lip_thickness, t=pin_tolerance, side=false);
				}
				//diameter
				else{
					pin(h=carrier_thickness+carrier_gear_gap, r=pin_diameter/2, lh=pin_lip_height, lt=pin_lip_thickness, t=pin_tolerance, side=false);
				}
			}
		}
		rotate([0,0,i*ns/m*360/np-phi*(ns+np)/np-phi]){
		//Annuler 
			difference(){
				herringbone(np,pitch,P,DR,tol,helix_angle,T);
				herringbone(nr_b,pitch_b,P_b,DR_b,-tol,helix_angle_b,T+0.2);
			}
		//sun
		difference(){
			rotate([0,0,(np_b+1)*180/ns_b+phi*(ns_b+np_b)*2/ns_b])
				mirror([0,1,0])
					herringbone(ns_b,pitch_b,P_b,DR_b,tol,helix_angle_b,T);
			translate([0,0,-T/2])
				cylinder(r=w_b/sqrt(3),h=T,center=true,$fn=6);
		}	
		//planets
		for(i=[1:m_b])rotate([0,0,i*360/m_b+phi])translate([pitchD_b/2*(ns_b+np_b)/nr_b,0,0])
			rotate([0,0,i*ns_b/m_b*360/np_b-phi*(ns_b+np_b)/np_b-phi])
			herringbone(np_b,pitch_b,P_b,DR_b,tol,helix_angle_b,T);

	}
	} 
}
}

//Carrier
if (object==1 || object==2){
translate([0,0,carrier_translate_distance])
	difference(){
		union(){
			translate([0,0,carrier_thickness])
			cylinder(h=carrier_cylinder_height,r=carrier_cylinder_radius,$fn=50);
			linear_extrude(height = carrier_thickness, center = false, convexity = 10, twist = 0){
				minkowski(){
					circle(r = carrier_outer_fillet_radius,$fn=50);
					union(){
						for(i=[1:m])
							rotate([0,0,i*360/m+phi])
								translate([(pitchD/2*(ns+np)/nr+carrier_extra_length-carrier_outer_fillet_radius)/2,0,0])
							square ([cl,cw],center = true);
					}
				}

				for(i=[1:m]){
				//Fillet
					difference(){
					polygon(points=[[-((carrier_inner_fillet_radius+carrier_arm_width/2)*(cos(i*360/m+phi)*cos(i*360/m+180/m+phi)+sin(i*360/m+phi)*sin(i*360/m+180/m+phi))/(sin(i*360/m+phi)*cos(i*360/m+180/m+phi)-cos(i*360/m+phi)*sin(i*360/m+180/m+phi)))*cos(i*360/m+phi)-(carrier_arm_width/2)*sin(i*360/m+phi),-((carrier_inner_fillet_radius+carrier_arm_width/2)*(cos(i*360/m+phi)*cos(i*360/m+180/m+phi)+sin(i*360/m+phi)*sin(i*360/m+180/m+phi))/(sin(i*360/m+phi)*cos(i*360/m+180/m+phi)-cos(i*360/m+phi)*sin(i*360/m+180/m+phi)))*sin(i*360/m+phi)+(carrier_arm_width/2)*cos(i*360/m+phi)],[-((carrier_inner_fillet_radius+carrier_arm_width/2)*(cos(i*360/m+phi)*cos(i*360/m+180/m+phi)+sin(i*360/m+phi)*sin(i*360/m+180/m+phi))/(sin(i*360/m+phi)*cos(i*360/m+180/m+phi)-cos(i*360/m+phi)*sin(i*360/m+180/m+phi)))*cos((i+1)*360/m+phi)+(carrier_arm_width/2)*sin((i+1)*360/m+phi),-((carrier_inner_fillet_radius+carrier_arm_width/2)*(cos(i*360/m+phi)*cos(i*360/m+180/m+phi)+sin(i*360/m+phi)*sin(i*360/m+180/m+phi))/(sin(i*360/m+phi)*cos(i*360/m+180/m+phi)-cos(i*360/m+phi)*sin(i*360/m+180/m+phi)))*sin((i+1)*360/m+phi)-(carrier_arm_width/2)*cos((i+1)*360/m+phi)],[0,0]],paths=[[0,1,2]]);

					translate([((carrier_inner_fillet_radius+carrier_arm_width/2)*(pow(sin(i*360/m+phi),2)+pow(cos(i*360/m+phi),2))/(sin((i*360/m+phi)+180/m)*cos(i*360/m+phi)-cos((i*360/m)+180/m+phi)*sin(i*360/m+phi)))*cos(i*360/m+180/m+phi),((carrier_inner_fillet_radius+carrier_arm_width/2)*(pow(sin(i*360/m+phi),2)+pow(cos(i*360/m+phi),2))/(sin((i*360/m)+180/m+phi)*cos(i*360/m+phi)-cos((i*360/m)+180/m+phi)*sin(i*360/m+phi)))*sin(i*360/m+180/m+phi),0])
						circle(r = carrier_inner_fillet_radius,$fn=50);

					}
				}
			}
		}
		union(){
			for(i=[1:m])
				rotate([0,0,i*360/m+phi])
					translate([pitchD/2*(ns+np)/nr,0,0])
					{
						if (pin_diameter==0)
						{
							
						}
						else{
							//auto. max
							if (pin_diameter==-1)//
							{
								pinhole(h=carrier_thickness, r=Bearing_sun_root_radius, lh=pin_lip_height, lt=pin_lip_thickness, t=pin_tolerance,	tight=true,fixed=true);
							}
							//diameter
							else{
								pinhole(h=carrier_thickness, r=pin_diameter/2, lh=pin_lip_height, lt=pin_lip_thickness, t=pin_tolerance,	tight=true,fixed=true);
							}
						}
					}
			cylinder(r=w_c/sqrt(3),h=carrier_thickness+carrier_cylinder_height+1,center=false,$fn=6);
		}
	}

}



module rack(
	number_of_teeth=15,
	circular_pitch=10,
	pressure_angle=28,
	helix_angle=0,
	clearance=0,
	gear_thickness=5,
	flat=false){
addendum=circular_pitch/(4*tan(pressure_angle));

flat_extrude(h=gear_thickness,flat=flat)translate([0,-clearance*cos(pressure_angle)/2])
	union(){
		translate([0,-0.5-addendum])square([number_of_teeth*circular_pitch,1],center=true);
		for(i=[1:number_of_teeth])
			translate([circular_pitch*(i-number_of_teeth/2-0.5),0])
			polygon(points=[[-circular_pitch/2,-addendum],[circular_pitch/2,-addendum],[0,addendum]]);
	}
}

module monogram(h=1)
linear_extrude(height=h,center=true)
translate(-[3,2.5])union(){
	difference(){
		square([4,5]);
		translate([1,1])square([2,3]);
	}
	square([6,1]);
	translate([0,2])square([2,1]);
}

module herringbone(
	number_of_teeth=15,
	circular_pitch=10,
	pressure_angle=28,
	depth_ratio=1,
	clearance=0,
	helix_angle=0,
	gear_thickness=5){
union(){
	gear(number_of_teeth,
		circular_pitch,
		pressure_angle,
		depth_ratio,
		clearance,
		helix_angle,
		gear_thickness/2);
	mirror([0,0,1])
		gear(number_of_teeth,
			circular_pitch,
			pressure_angle,
			depth_ratio,
			clearance,
			helix_angle,
			gear_thickness/2);
}}

module gear (
	number_of_teeth=15,
	circular_pitch=10,
	pressure_angle=28,
	depth_ratio=1,
	clearance=0,
	helix_angle=0,
	gear_thickness=5,
	flat=false){
pitch_radius = number_of_teeth*circular_pitch/(2*PI);
twist=tan(helix_angle)*gear_thickness/pitch_radius*180/PI;

flat_extrude(h=gear_thickness,twist=twist,flat=flat)
	gear2D (
		number_of_teeth,
		circular_pitch,
		pressure_angle,
		depth_ratio,
		clearance);
}

module flat_extrude(h,twist,flat){
	if(flat==false)
		linear_extrude(height=h,twist=twist,slices=twist/6)child(0);
	else
		child(0);
}

module gear2D (
	number_of_teeth,
	circular_pitch,
	pressure_angle,
	depth_ratio,
	clearance){
pitch_radius = number_of_teeth*circular_pitch/(2*PI);
base_radius = pitch_radius*cos(pressure_angle);
depth=circular_pitch/(2*tan(pressure_angle));
outer_radius = clearance<0 ? pitch_radius+depth/2-clearance : pitch_radius+depth/2;
root_radius1 = pitch_radius-depth/2-clearance/2;
root_radius = (clearance<0 && root_radius1<base_radius) ? base_radius : root_radius1;
backlash_angle = clearance/(pitch_radius*cos(pressure_angle)) * 180 / PI;
half_thick_angle = 90/number_of_teeth - backlash_angle/2;
pitch_point = involute (base_radius, involute_intersect_angle (base_radius, pitch_radius));
pitch_angle = atan2 (pitch_point[1], pitch_point[0]);
min_radius = max (base_radius,root_radius);

intersection(){
	rotate(90/number_of_teeth)
		circle($fn=number_of_teeth*1,r=pitch_radius+depth_ratio*circular_pitch/2-clearance/2);
	union(){
		rotate(90/number_of_teeth)
			circle($fn=number_of_teeth*1,r=max(root_radius,pitch_radius-depth_ratio*circular_pitch/2-clearance/2));
		for (i = [1:number_of_teeth])rotate(i*360/number_of_teeth){
			halftooth (
				pitch_angle,
				base_radius,
				min_radius,
				outer_radius,
				half_thick_angle);		
			mirror([0,1])halftooth (
				pitch_angle,
				base_radius,
				min_radius,
				outer_radius,
				half_thick_angle);
		}
	}
}}

module halftooth (
	pitch_angle,
	base_radius,
	min_radius,
	outer_radius,
	half_thick_angle){
index=[0,1,2,3,4,5];
start_angle = max(involute_intersect_angle (base_radius, min_radius)-5,0);
stop_angle = involute_intersect_angle (base_radius, outer_radius);
angle=index*(stop_angle-start_angle)/index[len(index)-1];
p=[[0,0],
	involute(base_radius,angle[0]+start_angle),
	involute(base_radius,angle[1]+start_angle),
	involute(base_radius,angle[2]+start_angle),
	involute(base_radius,angle[3]+start_angle),
	involute(base_radius,angle[4]+start_angle),
	involute(base_radius,angle[5]+start_angle)];

difference(){
	rotate(-pitch_angle-half_thick_angle)polygon(points=p);
	square(2*outer_radius);
}}

// Mathematical Functions
//===============

// Finds the angle of the involute about the base radius at the given distance (radius) from it's center.
//source: http://www.mathhelpforum.com/math-help/geometry/136011-circle-involute-solving-y-any-given-x.html

function involute_intersect_angle (base_radius, radius) = sqrt (pow (radius/base_radius, 2) - 1) * 180 / PI;

// Calculate the involute position for a given base radius and involute angle.

function involute (base_radius, involute_angle) =
[
	base_radius*(cos (involute_angle) + involute_angle*PI/180*sin (involute_angle)),
	base_radius*(sin (involute_angle) - involute_angle*PI/180*cos (involute_angle))
];
