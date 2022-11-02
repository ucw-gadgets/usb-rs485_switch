pcb_w = 175;
pcb_d = 60;
pcb_h = 20;

pcb_t = 1.6;

pcb_solder = 2.5;

conn_w = 16.4;
conn_spc = 3.2;
conn_off = 10.6;
conn_off_2 = 11.1;
conn_h = 13.5;
conn_b = 2.5;
conn_d = 15.3;
conn_so = 2.2;

wall = 2;
spc = 0.4;

hole_corr = 0.4;
hole_off = 4.8;
hole_d = 2.8 + hole_corr;
hole_d2 = 3.2 + hole_corr;
hole_wall = 3.4;
hole_head = 2.2;

max = 1000;

pwr_w = 19.2;
pwr_h = 7.4;
pwr_off = 10.6;
pwr_d = 6;

usb_w = 10;
usb_h = 4.2;
usb_off = 52.3;
usb_d = 6;

$fn = 64;
$fs = 0.05;

text_d = 0.3;
fix = 0.2;

main_text = "RS-485 Switch";
main_font = "Liberation Sans:style=Bold";
main_size = 12;

port_font = "Liberation Sans:style=Bold";
port_size = 4;

module inside(){
	translate([0,0,0.1]) cube([pcb_d +2*spc, pcb_w  + 2*spc, pcb_h + 2*spc]);
}

module basebox() {
	difference() {
		minkowski() {
			//cube([wall * 2,wall * 2, wall * 2], center=true);
			cylinder(r = wall, h = 2*wall, center=true);
			inside();
		}
		inside();
	}
	translate([0, conn_off - wall, spc + pcb_solder]) cube([conn_d - conn_so - 2, pcb_w - conn_off + wall - conn_off_2 + wall, pcb_h - pcb_solder + spc + fix]);
	
	translate([0, 0, 0]) cube([hole_off + hole_wall, hole_off + hole_wall, pcb_h + 2 * spc + fix]);
	translate([0, pcb_w - (hole_off + hole_wall), 0]) cube([hole_off + hole_wall + 2 * spc, hole_off + hole_wall + 2 * spc, pcb_h + 2 * spc + fix]);
	
	translate([pcb_d - (hole_off + hole_wall), 0, 0]) cube([hole_off + hole_wall + 2 * spc, hole_off + hole_wall + 2 * spc, pcb_h + 2 * spc + fix]);
	translate([pcb_d - (hole_off + hole_wall), pcb_w - (hole_off + hole_wall), 0]) cube([hole_off + hole_wall + 2 * spc, hole_off + hole_wall+ 2 * spc, pcb_h + 2 * spc + fix]);
}

module pcb() {
	cube([pcb_d, pcb_w, pcb_t]);
	for (i = [0:7]) {
		translate([-conn_so, conn_off + i * (conn_w + conn_spc), pcb_t]) cube([conn_d, conn_w, conn_h]);
	}
	translate([pcb_d - 0.1, pwr_off, pcb_t]) cube([pwr_d, pwr_w, pwr_h]);
	
	translate([pcb_d - 0.1, usb_off, pcb_t]) {
		translate([0, usb_h/2, usb_h / 2]) rotate([0,90,0]) cylinder(d = usb_h, h = usb_d);
		translate([0, usb_w - usb_h/2, usb_h / 2]) rotate([0,90,0]) cylinder(d = usb_h, h = usb_d);
		translate([0, usb_h/2, 0]) cube([usb_d, usb_w - usb_h, usb_h]);
	}
						
	
}

module pcbspace() {
	minkowski() {
		cube([spc*2,spc*2,spc*2], center = true);
		translate([0, 0, pcb_solder]) pcb();
	}
}

module screw() {
	cylinder(d = hole_d, h = pcb_h + 2 * spc);
	cylinder(d = hole_d2, h = wall + hole_d2);
	cylinder(d2 = hole_d2, d1 = hole_d2 + 2 * hole_head, h = hole_head);	
}

module box() {
	difference() {
		basebox();
		pcbspace();
		translate([hole_off, hole_off, - wall]) screw();
		translate([hole_off, pcb_w - hole_off, - wall]) screw();
		translate([pcb_d - hole_off, hole_off, - wall]) screw();
		translate([pcb_d - hole_off, pcb_w - hole_off, - wall]) screw();
	}
}

module split()
{
	translate([-wall, -wall, -wall]) cube([pcb_d + 2 * (wall + spc), pcb_w + 2 * (wall + spc), pcb_t + wall/2 + spc]);
	translate([-wall , -wall/2 , - wall/2 - spc ]) cube([pcb_d + 2*(wall+spc) - wall/2, pcb_w + 2 * (wall+spc) - wall , pcb_solder + pcb_t + wall/2 + spc]);
}
	

module markings()
{
	translate([pcb_d / 2 - 0.254 * main_size, pcb_w / 2, pcb_h + 2*spc + wall - text_d])
		rotate([0, 0, -90])
			linear_extrude(height = text_d + 0.1)
				text(text = main_text, font = main_font, size = main_size, halign = "center");

	for (i = [0:7])
		translate([- wall + 10, conn_off + i * (conn_w + conn_spc) + conn_w / 2, 18.5]) 
			rotate([90, 0, 0])
				rotate([0, -90, 0])
					linear_extrude(height = 10 + 0.1)
						scale([1.4, 1, 1]) 
							text(text = str(8 - i), font = port_font, size = port_size, halign = "center");
	
	translate([ -wall + 5, conn_off, 18.5])
		cube([5, 8 * (conn_w + conn_spc), 3.9]);
}

module topbox() {
	difference() {
		box();
		split();
		
	}
}

module botbox() {
	difference() {
		box();
		topbox();
	}
}

module topbox_t() {
	difference() {
		topbox();
		markings();
	}
}


markings();
//topbox_t();
//botbox();




