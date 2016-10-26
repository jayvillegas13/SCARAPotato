/*ARM CLASS*/
public void directKinematic(){

        // midpoint between joints
        double xa = xj1 + 0.5*(xj2 - xj1);
        double ya = yj1 + 0.5*(yj2 - yj1);
        // distance between joints
        double d = ((xj2- xj1) - (yj2 - yj1));
        if (d<2*r){
            valid_state = true;
            // half distance between tool positions
            double h = Math.sqrt((Math.pow(r,2)) - (Math.pow(d/2,2)));
            double alpha = Math.atan((yj1 - yj2)/(xj2 - xj1));
            // tool position
            double xt = xa + h*Math.cos(alpha-Math.PI/2);;
            double yt = ya + h*Math.sin(alpha-Math.PI/2);;
            double xt2 = xa - h*Math.cos(alpha-Math.PI/2);
            double yt2 = ya - h*Math.sin(alpha-Math.PI/2);
        } else {
            valid_state = false;
        }
    }
    
    public void inverseKinematic(double xt_new,double yt_new){
        valid_state = true;

        xt = xt_new;
        yt = yt_new;

        valid_state = true;
        double dx1 = xt - xm1; 
        double dy1 = yt - ym1;

        // distance between pem and motor
        double d1 = Math.sqrt((Math.pow(dx1,2)) + (Math.pow(dy1,2)));
        if (d1>2*r){
            UI.println("Arm 1 - can not reach");
            valid_state = false;
            return;
        }

        double l1 = d1/2;
        double h1 = Math.sqrt(r*r - d1*d1/4);

        double theta11 = Math.atan2(yt-ym1, xm1-xt);
        double theta12 = ((Math.PI)/2 - theta11);
        double alphaX1 = xm1 + 0.5*(xt-xm1);
        double alphaY1 = ym1 + 0.5*(yt-ym1);
        double alpha = Math.atan((yj1 - yj2)/(xj2 - xj1));
        // elbows positions
        xj1 = alphaX1 + h1*(Math.cos(theta12));
        yj1 = alphaY1 + h1*(Math.sin(theta12));
        theta1 = Math.atan2((yj1 - ym1),(xj1 - xm1));
        if ((theta1 > 0)||(theta1 < -Math.PI)){
            valid_state = false;
            UI.println("Ange 1 -invalid");
            return;
        }

        //theta12 = Math.atan2(yj12 - ym1,xj12-xm1);
        double dx2 = xt - xm2; 
        double dy2 = yt - ym2;
        double d2 = Math.sqrt((Math.pow(dx2,2)) + (Math.pow(dy2,2)));
        if (d2>2*r){
            UI.println("Arm 2 - can not reach");
            valid_state = false;
            return;
        }

        double l2 = d2/2;

        double h2 = Math.sqrt(r*r - d2*d2/4);
        double theta21 = Math.atan2(yt-ym2, xm2-xt);
        double theta22 = ((Math.PI)/2 - theta21);
        double alphaX2 = xm2 + 0.5*(xt-xm2);
        double alphaY2 = ym2 + 0.5*(yt-ym2);
        // elbows positions
        UI.drawImage("elf.jpg", 200, 70);
        xj2 = alphaX2 - h2*(Math.cos(theta22));
        yj2 = alphaY2 - h2*(Math.sin(theta22));
        // motor angles for both 1st elbow positions
        theta2 = Math.atan2((yj2 - ym2),(xj2 - xm2));
        if ((theta2 > 0)||(theta2 < -Math.PI)){
            valid_state = false;
            UI.println("Ange 2 -invalid");
            return;
        }
        
        double d3 = ym1 + (ym2 - ym1)/2 - Math.sqrt(r*r - Math.pow(r-xm1+(xm2-xm1)/2, 2));
        if(yt > d3){
            valid_state = false;
            UI.println("Singularity");
            return;
        }
        
        UI.printf("xt:%3.1f, yt:%3.1f\n",xt,yt);
        UI.printf("theta1:%3.1f, theta2:%3.1f\n",theta1*180/Math.PI,theta2*180/Math.PI);
        return;
    }
    
    public int get_pwm1(){
        int pwm = 0;
        //pwm = (int)(pwm1_val_1 + ((theta1 - theta1_val_1)*(pwm1_val_1 - pwm1_val_2))/(theta1_val_2 - theta1_val_1));
        pwm = (int)(-10 * theta1 + 180);
        return pwm;
    }

    // ditto for motor 2
    public int get_pwm2(){
        int pwm = 0;
        //pwm = (int)(pwm2_val_1 + ((theta2 - theta2_val_1)*(pwm2_val_1 - pwm2_val_2))/(theta2_val_2 - theta2_val_1));             
        pwm = (int)(-10 * theta2 + 880);
        return pwm;
    }

    public int get_pwm3(){
        int pwm = 1999;
        return pwm;
    }
    
    /*MAIN CLASS*/
    public void save_pwm(){
        tool_path = new ToolPath();
        if(tool_path == null) UI.println("Cannot save");
        tool_path.convert_drawing_to_angles(drawing,arm," ");
        tool_path.convert_angles_to_pwm(arm);
        tool_path.save_pwm_file();
    }
    
    
    /*TOOLPATH CLASS*/
    public void convert_drawing_to_angles(Drawing drawing,Arm arm,String fname){
        // for all points of the drawing...        
        for (int i = 0;i < drawing.get_drawing_size()-1;i++){ 
            // take two points
            PointXY p0 = drawing.get_drawing_point(i);
            PointXY p1 = drawing.get_drawing_point(i+1);
            // break line between points into segments: n_steps of them
            for ( int j = 0 ; j< n_steps;j++) { // break segment into n_steps str. lines
                double x = p0.get_x() + j*(p1.get_x()-p0.get_x())/n_steps;
                double y = p0.get_y() + j*(p1.get_y()-p0.get_y())/n_steps;
                arm.inverseKinematic(x, y);
                theta1_vector.add(arm.get_theta1()*180/Math.PI);
                theta2_vector.add(arm.get_theta2()*180/Math.PI);
                if (p1.get_pen()){ 
                    pen_vector.add(1);
                } else {
                    pen_vector.add(3);
                }
            }
        }
        save_angles(fname);
    }

    public void save_angles(String fname){
        for ( int i = 0 ; i < theta1_vector.size(); i++){
            UI.printf(" t1=%3.1f t2=%3.1f pen=%d\n",
                theta1_vector.get(i),theta2_vector.get(i),pen_vector.get(i));
        }

        try {
            //Whatever the file path is.
            File statText = new File(fname);
            FileOutputStream is = new FileOutputStream(statText);
            OutputStreamWriter osw = new OutputStreamWriter(is);    
            Writer w = new BufferedWriter(osw);
            String str_out;
            for (int i = 1; i < theta1_vector.size() ; i++){
                str_out = String.format("%3.1f,%3.1f,%d\r\n",
                    theta1_vector.get(i),theta2_vector.get(i),pen_vector.get(i));
                w.write(str_out);
            }
            w.close();
        } catch (IOException e) {
            UI.println("Problem writing to the file statsTest.txt");
        }

    }

    // takes sequence of angles and converts it 
    // into sequence of motor signals
    public void convert_angles_to_pwm(Arm arm){
        // for each angle
        for (int i=0 ; i < theta1_vector.size();i++){
            arm.set_angles(theta1_vector.get(i),theta2_vector.get(i));
            int pwm1 = arm.get_pwm1();
            int pwm2 = arm.get_pwm2();
            if(pwm1 > 1000 && pwm1 < 2000){
                pwm1_vector.add(arm.get_pwm1());
            }
            if(pwm2 > 1000 && pwm2 < 2000){
                pwm2_vector.add(arm.get_pwm2());
            }
            if(pen_vector.get(i) == 1){
                pwm3_vector.add(1700);
            } else {
                pwm3_vector.add(1100);
            }
        }
    }

    // save file with motor control values
    public void save_pwm_file(){
        String filename = UIFileChooser.save();
        if(filename!=null){
            try {
                PrintStream print = new PrintStream(new File(filename));
                for (int i = 0; i < pwm1_vector.size() ; i++){
                    int pwm1 = pwm1_vector.get(i);
                    int pwm2 = pwm2_vector.get(i);
                    int pwm3 = pwm3_vector.get(i);

                    String pm1 = pwm1 +"";
                    String pm2 = pwm2 +"";
                    String pm3 = pwm3 +"";
                    print.println(pm1+","+pm2+","+pm3);
                }
                print.close();
            } catch (IOException e) {
                UI.printf("Saving fail % \n", e);
            }
        }
    }
    
    /*DRAWING CLASS*/
    public void save_path(String fname){
        try {
            //Whatever the file path is.
            File statText = new File(fname);
            FileOutputStream is = new FileOutputStream(statText);
            OutputStreamWriter osw = new OutputStreamWriter(is);
            Writer w = new BufferedWriter(osw);
            String str_out; 
            for (int i = 1; i < path.size() ; i++){
                if (path.get(i).get_pen()) {
                  str_out = path.get(i).get_x() +" "+ path.get(i).get_y() +" 1\r\n";
                } else {
                  str_out = path.get(i).get_x() +" "+ path.get(i).get_y() +" 0\r\n";
                }
                w.write(str_out);
            }
            w.close();
        } catch (IOException e) {
            UI.println("Problem writing to the file statsTest.txt");
        }
    }
