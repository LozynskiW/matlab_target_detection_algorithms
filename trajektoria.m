clc

X=out.x;
Y=out.y;
Z=out.z;
Fi=out.fi; %yaw X
Teta=out.teta; %pitch Y
Gamma=out.gamma; %roll Z

%Korekcja do modelu
%{
%Teta=Teta+4.7124;
%Fi=Fi+3.14/2;
Teta=-1*Teta;
Fi=-1*Fi;
%}

plot3(X,Y,Z)
xlabel('x(t)')
ylabel('y(t)')
zlabel('z(t)')


%% I sposób
%{
skrypt = fopen('symulacja_2.py','w');
klatka='bpy.context.scene.frame_current = %d\n';
translacja="bpy.ops.transform.translate(value=(%4.2f,%4.2f,%4.2f),orient_type='GLOBAL', orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)),orient_matrix_type='GLOBAL', constraint_axis=(True, False, False),mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH',proportional_size=7.40025, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
rotacjaX="bpy.ops.transform.rotate(value=%4.4f, orient_axis='X', orient_type='LOCAL', orient_matrix=((%4.4f, %4.4f, %4.4f), (%4.4f, %4.4f, %4.4f), (%4.4f, %4.4f, %4.4f)), orient_matrix_type='LOCAL', constraint_axis=(True, False, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=7.40025, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
rotacjaY="bpy.ops.transform.rotate(value=%4.4f, orient_axis='Y', orient_type='LOCAL', orient_matrix=((%4.4f, %4.4f, %4.4f), (%4.4f, %4.4f, %4.4f), (%4.4f, %4.4f, %4.4f)), orient_matrix_type='LOCAL', constraint_axis=(False, True, False), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=7.40025, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";
rotacjaZ="bpy.ops.transform.rotate(value=%4.4f, orient_axis='Z', orient_type='LOCAL', orient_matrix=((%4.4f, %4.4f, %4.4f), (%4.4f, %4.4f, %4.4f), (%4.4f, %4.4f, %4.4f)), orient_matrix_type='LOCAL', constraint_axis=(False, False, True), mirror=True, use_proportional_edit=False, proportional_edit_falloff='SMOOTH', proportional_size=7.40025, use_proportional_connected=False, use_proportional_projected=False, release_confirm=True)\n";

        yaw=0;
        pitch=0;
        roll=0;

        Rx=[1 0 0;
            0 cos(yaw) -sin(yaw);
            0 sin(yaw) cos(yaw)];
        Ry=[cos(pitch) 0 sin(pitch);
            0 1 0;
            -sin(pitch) 0 cos(pitch)];
        Rz=[cos(roll) -sin(roll) 0;
            sin(roll) cos(roll) 0;
            0 0 1];
        R0=Rx*Ry*Rz;
        
for i=1:1:length(X)
    if i==1
        fprintf(skrypt,'import bpy\n');  
        fprintf(skrypt,'ob = bpy.context.scene.objects["Samolot"]\n');       
        fprintf(skrypt,"bpy.ops.object.select_all(action='DESELECT')\n"); 
        fprintf(skrypt,'bpy.context.view_layer.objects.active = ob\n');    
        fprintf(skrypt,'ob.select_set(True)\n');
        fprintf(skrypt,'bpy.context.scene.tool_settings.use_keyframe_insert_auto = True\n');
        R_prev=R0;
        x=X(i,1);
        y=Y(i,1);
        z=Z(i,1);
        fprintf(skrypt,klatka,i);
        fprintf(skrypt,translacja,x,y,z);
        % Obrót po X
        yaw=Fi(i,1);
        pitch=0;
        roll=0;
        fprintf(skrypt,rotacjaX,yaw,R_prev(1,1),R_prev(1,2),R_prev(1,3),R_prev(2,1),R_prev(2,2),R_prev(2,3),R_prev(3,1),R_prev(3,2),R_prev(3,3));

        Rx=[1 0 0;
            0 cos(yaw) -sin(yaw);
            0 sin(yaw) cos(yaw)];
        Ry=[cos(pitch) 0 sin(pitch);
            0 1 0;
            -sin(pitch) 0 cos(pitch)];
        Rz=[cos(roll) -sin(roll) 0;
            sin(roll) cos(roll) 0;
            0 0 1];
        R1=Rx*Ry*Rz;
        R1=R_prev*R1;
        R1=R1';
        % Obrót po Y
        yaw=0;
        pitch=Teta(i,1);
        roll=0;
        fprintf(skrypt,rotacjaY,pitch,R1(1,1),R1(1,2),R1(1,3),R1(2,1),R1(2,2),R1(2,3),R1(3,1),R1(3,2),R1(3,3));
        Rx=[1 0 0;
            0 cos(yaw) -sin(yaw);
            0 sin(yaw) cos(yaw)];
        Ry=[cos(pitch) 0 sin(pitch);
            0 1 0;
            -sin(pitch) 0 cos(pitch)];
        Rz=[cos(roll) -sin(roll) 0;
            sin(roll) cos(roll) 0;
            0 0 1];
        R2=Rx*Ry*Rz;
        R2=R1*R2;
        R2=R2';
        
        % Obrót po Z
        yaw=0;
        pitch=0;
        roll=Gamma(i,1);
        fprintf(skrypt,rotacjaZ,roll,R2(1,1),R2(1,2),R2(1,3),R2(2,1),R2(2,2),R2(2,3),R2(3,1),R2(3,2),R2(3,3));
        Rx=[1 0 0;
            0 cos(yaw) -sin(yaw);
            0 sin(yaw) cos(yaw)];
        Ry=[cos(pitch) 0 sin(pitch);
            0 1 0;
            -sin(pitch) 0 cos(pitch)];
        Rz=[cos(roll) -sin(roll) 0;
            sin(roll) cos(roll) 0;
            0 0 1];
        R_prev=Rx*Ry*Rz;
        R_prev=R2*R_prev;
        R_prev=R_prev'; %Macierz rotacji dla przysz³ych pokoleñ 

    else
        x=X(i,1)-X(i-1,1);
        y=Y(i,1)-Y(i-1,1);
        z=Z(i,1)-Z(i-1,1);
        fprintf(skrypt,klatka,i);
        fprintf(skrypt,translacja,x,y,z);
        % Obrót po X
        yaw=Fi(i,1)-Fi(i-1,1);
        pitch=0;
        roll=0;
        fprintf(skrypt,rotacjaX,yaw,R_prev(1,1),R_prev(1,2),R_prev(1,3),R_prev(2,1),R_prev(2,2),R_prev(2,3),R_prev(3,1),R_prev(3,2),R_prev(3,3));

        Rx=[1 0 0;
            0 cos(pitch) -sin(pitch);
            0 sin(pitch) cos(pitch)];
        Ry=[cos(roll) 0 sin(roll);
            0 1 0;
            -sin(roll) 0 cos(roll)];
        Rz=[cos(yaw) -sin(yaw) 0;
            sin(yaw) cos(yaw) 0;
            0 0 1];
        R1=Rx*Ry*Rz;
        R1=R_prev*R1;
        R1=R1';
        % Obrót po Y
        yaw=0;
        pitch=Teta(i,1)-Teta(i-1,1);
        roll=0;
        fprintf(skrypt,rotacjaY,pitch,R1(1,1),R1(1,2),R1(1,3),R1(2,1),R1(2,2),R1(2,3),R1(3,1),R1(3,2),R1(3,3));
        Rx=[1 0 0;
            0 cos(pitch) -sin(pitch);
            0 sin(pitch) cos(pitch)];
        Ry=[cos(roll) 0 sin(roll);
            0 1 0;
            -sin(roll) 0 cos(roll)];
        Rz=[cos(yaw) -sin(yaw) 0;
            sin(yaw) cos(yaw) 0;
            0 0 1];
        R2=Rx*Ry*Rz;
        R2=R1*R2;
        R2=R2';
        
        % Obrót po Z
        yaw=0;
        pitch=0;
        roll=Gamma(i,1)-Gamma(i-1,1);
        fprintf(skrypt,rotacjaZ,roll,R2(1,1),R2(1,2),R2(1,3),R2(2,1),R2(2,2),R2(2,3),R2(3,1),R2(3,2),R2(3,3));
         Rx=[1 0 0;
            0 cos(pitch) -sin(pitch);
            0 sin(pitch) cos(pitch)];
        Ry=[cos(roll) 0 sin(roll);
            0 1 0;
            -sin(roll) 0 cos(roll)];
        Rz=[cos(yaw) -sin(yaw) 0;
            sin(yaw) cos(yaw) 0;
            0 0 1];
        R_prev=Rx*Ry*Rz;
        R_prev=R2*R_prev;
        R_prev=R_prev'; %Macierz rotacji dla przysz³ych pokoleñ         
    end
end
fclose(skrypt);
%}

%{
%% II sposób
skrypt = fopen('symulacja_2.py','w');
klatka='bpy.context.scene.frame_current = %d\n';
translacjaX='bpy.context.object.location[0] = %4.4f\n';
translacjaY='bpy.context.object.location[1] = %4.4f\n';
translacjaZ='bpy.context.object.location[2] = %4.4f\n';
rotacjaX='bpy.context.object.rotation_euler[0] = %4.4f\n';
rotacjaY='bpy.context.object.rotation_euler[1] = %4.4f\n';
rotacjaZ='bpy.context.object.rotation_euler[2] = %4.4f\n';
for i=1:1:length(X)
    if i==1
        fprintf(skrypt,'import bpy\n');  
        fprintf(skrypt,'ob = bpy.context.scene.objects["Samolot"]\n');       
        fprintf(skrypt,"bpy.ops.object.select_all(action='DESELECT')\n"); 
        fprintf(skrypt,'bpy.context.view_layer.objects.active = ob\n');    
        fprintf(skrypt,'ob.select_set(True)\n');
        fprintf(skrypt,'bpy.context.scene.tool_settings.use_keyframe_insert_auto = True\n');
        fprintf(skrypt,'orient_slot = bpy.context.scene.transform_orientation_slots[0]\n');
        fprintf(skrypt,'custom = orient_slot.custom_orientation\n');
        fprintf(skrypt,"bpy.context.scene.transform_orientation_slots[0].type = 'LOCAL'\n");
    end
% teta - pitch X
% gamma - roll Y
% fi - yaw Z
        fprintf(skrypt,klatka,i);
        fprintf(skrypt,translacjaX,X(i,1));
        fprintf(skrypt,translacjaY,Y(i,1));
        fprintf(skrypt,translacjaZ,Z(i,1));
        fprintf(skrypt,rotacjaX,Teta(i,1));
        fprintf(skrypt,rotacjaY,Gamma(i,1));
        fprintf(skrypt,rotacjaZ,Fi(i,1));
        fprintf(skrypt,"bpy.ops.anim.keyframe_insert_menu(type='Rotation')\n");
        fprintf(skrypt,"bpy.ops.anim.keyframe_insert_menu(type='Location')\n");
end
fclose(skrypt);
%}

%% III sposób

skrypt = fopen('symulacja_2.py','w');
translacja=fileread('komenda_py.txt');
fprintf(skrypt,'import bpy\n');  
fprintf(skrypt,'ob = bpy.context.scene.objects["TorRuchu"]\n');       
fprintf(skrypt,"bpy.ops.object.select_all(action='DESELECT')\n"); 
fprintf(skrypt,'bpy.context.view_layer.objects.active = ob\n');    
fprintf(skrypt,'ob.select_set(True)\n');
fprintf(skrypt,'bpy.ops.object.editmode_toggle()\n');
fprintf(skrypt,'bpy.ops.curve.de_select_last()\n');

for i=1:1:length(X)
    if i==1
        x=X(i,1);
        y=Y(i,1);
        z=Z(i,1);
    else
        x=X(i,1)-X(i-1,1);
        y=Y(i,1)-Y(i-1,1);
        z=Z(i,1)-Z(i-1,1);
    end
        fprintf(skrypt,translacja,x,y,z);

end
fclose(skrypt);
