// globals
$fn=100;
wall_w = 2;
sr_h = 46.5;
pcb_x = 35;
pcb_y = 35;
pcb_h2h = 25; 

module nema_17() {
    difference() {
        union() {
            translate([0,0,47/2]) cube([41, 41, 47], center=true); // main body
            translate([0,0,47-3]) cylinder(r=2.5, h=25); // shaft
            translate([0,0,47]) cylinder(r=12, h=3); // wide cylindrical indentation
        }
        union() {
            for(x_i = [-1,1], y_i = [-1,1]) {
                translate([15*x_i, 15*y_i, 47-5+0.1]) cylinder(r=1.5, h=5); // screw holes 
                translate([41/2*x_i,41/2*y_i,47/2-0.1]) rotate([0,0,45]) cube([5,5,47+0.3],center=true); // edges
                translate([41/2*x_i,41/2*y_i,23/2+12]) rotate([0,0,45]) cube([10,10,23],center=true);
            }

            difference() { // center subtraction (leads to the bearing)
                translate([0,0,47-2]) cylinder(r=4, h=5.1);
                translate([0,0,47-3]) cylinder(r=2.5, h=25);
            }
        }
    }
}

module sr_body(wire_r=1, od=27.5) {
    difference() {
        union() {
            translate([0,0,wall_w/2]) cube([41, 41, wall_w], center=true); // stepper body
            
            cylinder(r=od/2, h=sr_h+wall_w); // slip ring body
        }
        union() {
            for(x_i = [-1,1], y_i = [-1,1]) {
                translate([15*x_i, 15*y_i, -0.1]) cylinder(r=1.5, h=5.3); // screw holes 
                translate([41/2*x_i,41/2*y_i,47/2-0.1]) rotate([0,0,45]) cube([5,5,47+0.3],center=true); // edges
            }
            
            for(i=[0:4]) {
               translate([0,od/2+0.1, wire_r+wall_w+6.5+i*8.5]) rotate([90,0,0]) cylinder(r=wire_r, h=wall_w+0.2);
                // single wire holes
               translate([0,-od/2+wall_w+0.1, wire_r+wall_w+6.5+i*8.5]) rotate([90,0,0]) cylinder(r=wire_r, h=wall_w+0.2);
            }
            
            translate([od/2-wall_w-0.2,0,2*wire_r+wall_w]) rotate([0,90,0]) cylinder(r=2*wire_r, h=wall_w+0.3); // multiple wire exit holes
            translate([-(od/2-wall_w-0.2),0,2*wire_r+wall_w]) rotate([0,-90,0]) cylinder(r=2*wire_r, h=wall_w+0.3);
            
            
            translate([0,0,-0.1]) cylinder(r=od/2-wall_w, h=sr_h+wall_w+0.3); // wide cylindrical indentation
            
            
            
        }
    }
}

module shaft_attachment(id=5.4,od=18) {
    difference() {
        union() {
          cylinder(r=od/2, h=sr_h);
            
        }
        
        union() {
          translate([0,0,-0.1]) cylinder(r=id/2, h=sr_h+5+0.3);  
          translate([0.1,0,3+1.2]) rotate([0,90,0]) cylinder(r=1.2, h=od/2+0.3);
          
          for(i =  [0:2]) {  // screw holes (M2)
            rotate([0,0,i*360/3]) translate([0,(od+id)/4,sr_h-10+0.1]) cylinder(r=0.9, h=10);  
          }
          
          // 5.35, 6.1, 2.3
          translate([(od-id)/2,0,5.1-0.1]) cube([2.5,6.2,10.2+0.15], center=true);
       }
    }
}

module pcb_seat(od=pcb_x+30) {
    difference() {
        union() {
            cylinder(r=od/2,h=2*wall_w);
        }
        union() {
            for(i =  [0:2]) {  // screw holes (M2), 18 and 5.4 are the od and id, respectively of shaft_attachment
                rotate([0,0,i*360/3]) translate([0,(18+5.4)/4,-0.1]) cylinder(r=0.9, h=10);  
            }
        
            for(x_i = [-1,1], y_i = [-1,1]) {
                translate([x_i*pcb_h2h/2,y_i*pcb_h2h/2,-0.1]) cylinder(r=1.2, h=2*wall_w+0.3);
            }
            
            for(i = [-2:2]) {
                translate([pcb_x/2+2.5+(2+i)*2.1,i*2.54,-0.1]) cylinder(r=0.75, h=2*wall_w+0.3);
                translate([-(pcb_x/2+(-3+i)*-2.1),i*2.54,-0.1]) cylinder(r=0.75, h=2*wall_w+0.3);
            }
        }    
    }
}

module pcb() {
    difference() {
        union() {
            translate([0,0,wall_w/2]) cube([pcb_x, pcb_y, wall_w],center=true);
            
            
        }
        union() {
            for(x_i = [-1,1], y_i = [-1,1]) {
                translate([x_i*pcb_h2h/2,y_i*pcb_h2h/2,-0.1]) cylinder(r=1.6, h=wall_w+0.3);
            }
        }
    }
}

// translate([0,0,-47]) color([1,1,1]) nema_17();
 translate([0,0,5]) color([0.6, 0.8, 0.3, 0.8])  shaft_attachment();
// color([0.3, 0.6, 0.6, 0.5]) sr_body();
// translate([0,0,sr_h+5]) color([0.2, 0.7, 0.4, 0.8])  pcb_seat();
// translate([0,0,sr_h+5+2*wall_w]) color([1, 1, 1]) pcb();