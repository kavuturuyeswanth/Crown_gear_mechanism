															--CROWN GEAR MESHING A COSINE PINION 
-- Reference article to get cosine pinion equations: The generation principle and mathematical models of a novel cosine gear drive, Shanming Luo, 14, February, 2008

																	--FUNCTIONS--

--Function-1: cosine_pinion_maker
----This generates cosine pinion gear.
--1.coordinates of cosine pinion in YZ-plane-->extruded in X-direction.
--2.translated by in X-direction to achieve meshing position.
--3.If number of teeth of pinion is odd number, rotate the pinion by half tooth angle with respect to X-axis.
--4.create a hole at the centre of cosine_pinion.
function cosine_pinion_maker(m_n,z_pinion,b,crown_d_p)
	local cosine_pinion_points={}
	local dir_x=v(b+1, 0, 0)
	local index=1
	for n=0,(z_pinion), 0.02
	do
		theta=((2*math.pi)/z_pinion)*n
		y_cosine=((m_n*z_pinion/2)+(h_a*math.cos(z_pinion*theta)))*math.sin(theta) -- Reference article equation No: 8
		z_cosine=((m_n*z_pinion/2)+(h_a*math.cos(z_pinion*theta)))*math.cos(theta) -- Reference article equation No: 8
		cosine_pinion_points[index]=v(0,y_cosine,z_cosine ) --2D proile of cosine
		index=index+1
	end
	local cosine_pinion=linear_extrude(dir_x, cosine_pinion_points) -->Step-1.
	cosine_pinion=translate(crown_d_p/2-2,0,0)*cosine_pinion  -->step-2.
	index=1
    if z_pinion%2~=0 then  -- condition for odd teeth number.
    cosine_pinion=rotate(((360)/z_pinion)/2,X)*cosine_pinion  -->step-3.
    end
	hole=translate(crown_d_p/2-2,0,0)*rotate(90,Y)*cylinder(5,b+1) -->step-4.
	cosine_pinion=difference(cosine_pinion,hole) -->step-4.
	return cosine_pinion
end

--Function-2:crown_workpiece_twoteeth
----This function generates workpiece required for 2 teeth of crown gear.
--1.create inner 2D profile of workpiece on XY-plane and stored in crown_i_points.
--2.create outer 2D profile of workpiece on XY-plane and stored in crown_o_points.
--3.Extrude inner profile in Z-direction and translate in Z-diretion by 5 units.
--4.Extrude outer profile in Z-direction.
--5.translate Inner cylinder and Outer cylinder to meshing position with pinion 
--and implement difference operation betweeen outer cylinder and inner cylinder. This gives us workpiece for crown.
--6.create a hole at the centre of workpiece.
function crown_workpiece_twoteeth(h_c,h_a,b,z_crown,crown_d_p,cosine_d_p)
	local crown_i_points={v(0,0,0)}
	local crown_o_points={v(0,0,0)}
	local index=2
	local dir_z=v(0,0,h_c)
	for wrk_theta=(-2*math.pi/z_crown)*2,(0*math.pi/z_crown), 0.02*math.pi/180
	do
		crown_i_points[index]=v(((crown_d_p/2)-1)*math.cos(wrk_theta),((crown_d_p/2)-1)*math.sin(wrk_theta),0)  -->step-1.
		crown_o_points[index]=v(((crown_d_p/2)+b-1)*math.cos(wrk_theta),((crown_d_p/2)+b-1)*math.sin(wrk_theta),0) -->step-2.
		index=index+1
	end
	index=2
	crown_i=translate(0,0,5)*linear_extrude(dir_z,crown_i_points) -->step-3.
	crown_o=linear_extrude(dir_z,crown_o_points) -->step-4.
	crown=translate(0,0,-cosine_d_p/2-h_c+h_a)*difference(crown_o,crown_i) -->Step-5.
	hole=translate(0,0,-cosine_d_p/2-h_c+h_a)*cylinder(6,5) -->Step-6.
	crown=difference(crown,hole) -->Step-6.
	return crown
end

--Function-3: crown_generator
----This function generates 2 teeth of final crown gear. This 2 teeth are futher used to multiply to achieve desired number of teeth.
--1.rotate workpiece W.R.T Z-axis by 2% of an angle occupied by one teeth in a crown gear and store it in crown_rot_z.
--2.rotate cosine pinion W.R.T X-axis by tooth ratio* rotation angle of crown. This maintains rel. velcity with crown. Store it in shaper_rot_x.
--3.Execute difference of crown_rot_z and shaper_rot_x and store it in crown.
--4.store shapre_rot_x in cosine_pinion. This is to update rotated cosine pinion position.
--5.Repeat above steps "1-4" in a for loop ranging from 0-2 teeth. Maintain incremental value lessthan or equal to 0.02. 
--6. Store generated crown teeth in crown_basic.
function crown_generator(z_crown,z_pinion,crown,cosine_pinion)
	for s=0,2, 0.02
	do
		generation_angle=((2*math.pi)/z_crown)*s --cut_angle-->2% of a single tooth angle of crown gear.
		crown_rot_z=rotate(generation_angle,Z)*crown -->Step-1.
		shaper_rot_x=rotate(generation_angle*z_crown/z_pinion,X)*cosine_pinion -->Step-2.
		generation=difference(crown_rot_z,shaper_rot_x) -->Step-3
		crown=generation-->Step-3.	
		cosine_pinion=shaper_rot_x	-->Step-4.	   
	end
	crown_basic=crown -->Step-5.
	return crown,cosine_pinion,crown_basic 
end

--Function-4: crown_gear_desired_teeth
----This function multiplies teeth from "crown_generator" function into desired number of teeth of crown gear.
--1.rotate crown_basic by angle of "2" teeth of crown gear.
--2.Make union of existing crown and rotated crown_basic.
--3.increment the teeth_count by "2"
--4.Repeat above steps "1-3" iteratively using while loop until desired number of teeth are generated. 
function crown_gear_desired_teeth(crown_basic,z_crown,crown)
	local teeth_count=2
	while teeth_count<z_crown
	do  
		crown_rot_z=rotate((360/z_crown)*teeth_count,Z)*crown_basic -->Step-1.
		crown=union(crown,crown_rot_z) -->Step-2.
		teeth_count=teeth_count+2     -->Step-3. 
	end
	return crown
end

--Function-5: simulaton_machine_elements
--Motion is implemented with input parameter from user.
--Input parameter for motion--> motion angle
function simulaton_machine_elements(motion_angle,crown_gear,cosine_pinion,z_crown,z_pinion)
	local motion_angle=ui_numberBox("Simulation",0)*(360/z_crown)*0.02 --input parameter.
	crown_gear=rotate(motion_angle,Z)*crown_gear		--rotates crown_gear by input parameter.
	cosine_pinion=rotate(motion_angle*z_crown/z_pinion,X)*cosine_pinion  --rotates cosine_pinion by rel.velcotiy of crown gear.
	emit(cosine_pinion,1)
	emit(crown_gear,5)
end
                                                                -------------END OF FUNCTIONS--------------

																		--INPUT PARAMETERS--											
m_n=ui_text('Enter Module', "3")*1 --Module   Module should be same for meshing gears
b=ui_text('Enter Facewidth', "10")*1  --Face width of crown gear and cosine pinion is same
z_pinion=ui_numberBox("No of Teeth of Pinion Gear", 10.0)*1 --Number of teeth of cosine pinion
z_crown=ui_numberBox("No of Teeth of Crown Gear", 30.0)*1 --Number of teeth of crown
cosine_d_p=m_n*z_pinion --pitch diameter of cosine pinion
crown_d_p=m_n*z_crown  -- pitch diameter of crown

h_a=m_n  --amplitude of cosine profile.

h_c=2*h_a+7 --Height of workpiece of crown gear.Height is maintained below the dedendum of crown gear.

--COSINE PINION:
cosine_pinion=cosine_pinion_maker(m_n,z_pinion,b,crown_d_p)

--CROWN WORKPIECE FOR 2 TEETH:
crown=crown_workpiece_twoteeth(h_c,h_a,b,z_crown,crown_d_p,cosine_d_p)

--Note: Meshing position is achieved in such a way that Intersection of perpendicular axes of meshing gears is at origin'world coordinate system'. 
--CROWN GEAR WITH 2 TEETH:
crown,cosine_pinion,crown_basic=crown_generator(z_crown,z_pinion,crown,cosine_pinion)

--CROWN GEAR WITH DESIRED NUMBER OF TEETH:
crown_gear=crown_gear_desired_teeth(crown_basic,z_crown,crown)

---USER CONTROLLED SIMULATION OF MACHINE ELEMENTS:
simulaton_machine_elements(motion_angle,crown_gear,cosine_pinion,z_crown,z_pinion)

--screenshot()