function q = inverse_kinematic(robot,T)

%theta3
    x=T(1,4)-72*T(1,3);
    y=T(2,4)-72*T(2,3);
    z=T(3,4)-72*T(3,3);
    A=x^2+y^2+z^2-580*z-84904;

    theta_31=2*atan2(-326160-sqrt(112095705600-4*A^2),2*A+75600);
    theta_32=2*atan2(-326160+sqrt(112095705600-4*A^2),2*A+75600);

%theta2

    a1=302*sin(theta_31)-70*cos(theta_31)+20-z;
    b1=-140*sin(theta_31)-604*cos(theta_31);
    c1=-302*sin(theta_31)+70*cos(theta_31)+560-z;
    theta_21=2*atan2(-b1+sqrt(b1^2-4*a1*c1),2*a1);
    theta_22=2*atan2(-b1-sqrt(b1^2-4*a1*c1),2*a1);

    a2=302*sin(theta_32)-70*cos(theta_32)+20-z;
    b2=-140*sin(theta_32)-604*cos(theta_32);
    c2=-302*sin(theta_32)+70*cos(theta_32)+560-z;
    theta_23=2*atan2(-b2+sqrt(b2^2-4*a2*c2),2*a2);
    theta_24=2*atan2(-b2-sqrt(b2^2-4*a2*c2),2*a2);

%theta1
%{
    theta_11=acos(x/302*cos(theta_21+theta_31)+70*sin(theta_21+theta_31)+270*sin(theta_21));
    theta_12=acos(x/302*cos(theta_22+theta_31)+70*sin(theta_22+theta_31)+270*sin(theta_22));
    theta_13=acos(x/302*cos(theta_23+theta_32)+70*sin(theta_23+theta_32)+270*sin(theta_23));
    theta_14=acos(x/302*cos(theta_24+theta_32)+70*sin(theta_24+theta_32)+270*sin(theta_24));
%}
    theta_11=atan2(y,x);

%456

    R36_1=[cos(theta_11)*sin(theta_21+theta_31)*T(1,1)+sin(theta_11)*sin(theta_21+theta_31)*T(2,1)+cos(theta_21+theta_31)*T(3,1)  cos(theta_11)*sin(theta_21+theta_31)*T(1,2)+sin(theta_11)*sin(theta_21+theta_31)*T(2,2)+cos(theta_21+theta_31)*T(3,2)  cos(theta_11)*sin(theta_21+theta_31)*T(1,3)+sin(theta_11)*sin(theta_21+theta_31)*T(2,3)+cos(theta_21+theta_31)*T(3,3)
         sin(theta_11)*T(1,1)-cos(theta_11)*T(2,1)  sin(theta_11)*T(1,2)-cos(theta_11)*T(2,2)  sin(theta_11)*T(1,3)-cos(theta_11)*T(2,3)
         cos(theta_21)*cos(theta_21+theta_31)*T(1,1)+sin(theta_11)*cos(theta_21+theta_31)*T(2,1)-sin(theta_21+theta_31)*T(3,1)  cos(theta_21)*cos(theta_21+theta_31)*T(1,2)+sin(theta_11)*cos(theta_21+theta_31)*T(2,2)-sin(theta_21+theta_31)*T(3,2)  cos(theta_21)*cos(theta_21+theta_31)*T(1,3)+sin(theta_11)*cos(theta_21+theta_31)*T(2,3)-sin(theta_21+theta_31)*T(3,3)
    ];
    R36_2=[cos(theta_11)*sin(theta_22+theta_31)*T(1,1)+sin(theta_11)*sin(theta_22+theta_31)*T(2,1)+cos(theta_22+theta_31)*T(3,1)  cos(theta_11)*sin(theta_22+theta_31)*T(1,2)+sin(theta_11)*sin(theta_22+theta_31)*T(2,2)+cos(theta_22+theta_31)*T(3,2)  cos(theta_11)*sin(theta_22+theta_31)*T(1,3)+sin(theta_11)*sin(theta_22+theta_31)*T(2,3)+cos(theta_22+theta_31)*T(3,3)
         sin(theta_11)*T(1,1)-cos(theta_11)*T(2,1)  sin(theta_11)*T(1,2)-cos(theta_11)*T(2,2)  sin(theta_11)*T(1,3)-cos(theta_11)*T(2,3)
         cos(theta_22)*cos(theta_22+theta_31)*T(1,1)+sin(theta_11)*cos(theta_22+theta_31)*T(2,1)-sin(theta_22+theta_31)*T(3,1)  cos(theta_22)*cos(theta_22+theta_31)*T(1,2)+sin(theta_11)*cos(theta_22+theta_31)*T(2,2)-sin(theta_22+theta_31)*T(3,2)  cos(theta_22)*cos(theta_22+theta_31)*T(1,3)+sin(theta_11)*cos(theta_22+theta_31)*T(2,3)-sin(theta_22+theta_31)*T(3,3)
    ];
    R36_3=[cos(theta_11)*sin(theta_23+theta_32)*T(1,1)+sin(theta_11)*sin(theta_23+theta_32)*T(2,1)+cos(theta_23+theta_32)*T(3,1)  cos(theta_11)*sin(theta_23+theta_32)*T(1,2)+sin(theta_11)*sin(theta_23+theta_32)*T(2,2)+cos(theta_23+theta_32)*T(3,2)  cos(theta_11)*sin(theta_23+theta_32)*T(1,3)+sin(theta_11)*sin(theta_23+theta_32)*T(2,3)+cos(theta_23+theta_32)*T(3,3)
         sin(theta_11)*T(1,1)-cos(theta_11)*T(2,1)  sin(theta_11)*T(1,2)-cos(theta_11)*T(2,2)  sin(theta_11)*T(1,3)-cos(theta_11)*T(2,3)
         cos(theta_23)*cos(theta_23+theta_32)*T(1,1)+sin(theta_11)*cos(theta_23+theta_32)*T(2,1)-sin(theta_23+theta_32)*T(3,1)  cos(theta_23)*cos(theta_23+theta_32)*T(1,2)+sin(theta_11)*cos(theta_23+theta_32)*T(2,2)-sin(theta_23+theta_32)*T(3,2)  cos(theta_23)*cos(theta_23+theta_32)*T(1,3)+sin(theta_11)*cos(theta_23+theta_32)*T(2,3)-sin(theta_23+theta_32)*T(3,3)
    ];
    R36_4=[cos(theta_11)*sin(theta_24+theta_32)*T(1,1)+sin(theta_11)*sin(theta_24+theta_32)*T(2,1)+cos(theta_24+theta_32)*T(3,1)  cos(theta_11)*sin(theta_24+theta_32)*T(1,2)+sin(theta_11)*sin(theta_24+theta_32)*T(2,2)+cos(theta_24+theta_32)*T(3,2)  cos(theta_11)*sin(theta_24+theta_32)*T(1,3)+sin(theta_11)*sin(theta_24+theta_32)*T(2,3)+cos(theta_24+theta_32)*T(3,3)
         sin(theta_11)*T(1,1)-cos(theta_11)*T(2,1)  sin(theta_11)*T(1,2)-cos(theta_11)*T(2,2)  sin(theta_11)*T(1,3)-cos(theta_11)*T(2,3)
         cos(theta_24)*cos(theta_24+theta_32)*T(1,1)+sin(theta_11)*cos(theta_24+theta_32)*T(2,1)-sin(theta_24+theta_32)*T(3,1)  cos(theta_24)*cos(theta_24+theta_32)*T(1,2)+sin(theta_11)*cos(theta_24+theta_32)*T(2,2)-sin(theta_24+theta_32)*T(3,2)  cos(theta_24)*cos(theta_24+theta_32)*T(1,3)+sin(theta_11)*cos(theta_24+theta_32)*T(2,3)-sin(theta_24+theta_32)*T(3,3)
    ];

%theta5

    theta_51=atan2(real(sqrt(R36_1(1,3)^2+R36_1(2,3)^2)),real(R36_1(3,3)));
    theta_52=atan2(real(-sqrt(R36_1(1,3)^2+R36_1(2,3)^2)),real(R36_1(3,3)));
    theta_53=atan2(real(sqrt(R36_2(1,3)^2+R36_2(2,3)^2)),real(R36_2(3,3)));
    theta_54=atan2(real(-sqrt(R36_2(1,3)^2+R36_2(2,3)^2)),real(R36_2(3,3)));
    theta_55=atan2(real(sqrt(R36_3(1,3)^2+R36_3(2,3)^2)),real(R36_3(3,3)));
    theta_56=atan2(real(-sqrt(R36_3(1,3)^2+R36_3(2,3)^2)),real(R36_3(3,3)));
    theta_57=atan2(real(sqrt(R36_4(1,3)^2+R36_4(2,3)^2)),real(R36_4(3,3)));
    theta_58=atan2(real(-sqrt(R36_4(1,3)^2+R36_4(2,3)^2)),real(R36_4(3,3)));
  
%theta4

    theta_41=atan2(real(-R36_1(2,3)/sin(theta_51)),real(-R36_1(1,3)/sin(theta_51)));
    theta_42=atan2(real(-R36_1(2,3)/sin(theta_52)),real(-R36_1(1,3)/sin(theta_52)));
    theta_43=atan2(real(-R36_2(2,3)/sin(theta_53)),real(-R36_2(1,3)/sin(theta_53)));
    theta_44=atan2(real(-R36_2(2,3)/sin(theta_54)),real(-R36_2(1,3)/sin(theta_54)));
    theta_45=atan2(real(-R36_3(2,3)/sin(theta_55)),real(-R36_3(1,3)/sin(theta_55)));
    theta_46=atan2(real(-R36_3(2,3)/sin(theta_56)),real(-R36_3(1,3)/sin(theta_56)));
    theta_47=atan2(real(-R36_4(2,3)/sin(theta_57)),real(-R36_4(1,3)/sin(theta_57)));
    theta_48=atan2(real(-R36_4(2,3)/sin(theta_58)),real(-R36_4(1,3)/sin(theta_58)));


%theta6

    theta_61=atan2(real(-R36_1(3,2)/sin(theta_51)),real(R36_1(3,1)/sin(theta_51)));
    theta_62=atan2(real(-R36_1(3,2)/sin(theta_52)),real(R36_1(3,1)/sin(theta_52)));
    theta_63=atan2(real(-R36_2(3,2)/sin(theta_53)),real(R36_2(3,1)/sin(theta_53)));
    theta_64=atan2(real(-R36_2(3,2)/sin(theta_54)),real(R36_2(3,1)/sin(theta_54)));
    theta_65=atan2(real(-R36_3(3,2)/sin(theta_55)),real(R36_3(3,1)/sin(theta_55)));
    theta_66=atan2(real(-R36_3(3,2)/sin(theta_56)),real(R36_3(3,1)/sin(theta_56)));
    theta_67=atan2(real(-R36_4(3,2)/sin(theta_57)),real(R36_4(3,1)/sin(theta_57)));
    theta_68=atan2(real(-R36_4(3,2)/sin(theta_58)),real(R36_4(3,1)/sin(theta_58)));

%end
    qx=zeros(8,6);
    qx(1,:)=real([theta_11,theta_21,theta_31,theta_41,theta_51,theta_61]);
    qx(2,:)=real([theta_11,theta_21,theta_31,theta_42,theta_52,theta_62]);
    qx(3,:)=real([theta_11,theta_22,theta_31,theta_43,theta_53,theta_63]);
    qx(4,:)=real([theta_11,theta_22,theta_31,theta_44,theta_54,theta_64]);
    qx(5,:)=real([theta_11,theta_23,theta_32,theta_45,theta_55,theta_65]);
    qx(6,:)=real([theta_11,theta_23,theta_32,theta_46,theta_56,theta_66]);
    qx(7,:)=real([theta_11,theta_24,theta_32,theta_47,theta_57,theta_67]);
    qx(8,:)=real([theta_11,theta_24,theta_32,theta_48,theta_58,theta_68]);

    stander=0.6;

    for i=1:8
        if all(abs( T- forward_kinematics(robot,qx(i,:))) < stander)
            q=qx(i,:);
        end
    end

    if q(1)>165*pi/180
        q(1)=q(1)-2*pi;
    end
    if q(1)<-165*pi/180
        q(1)=q(1)+2*pi;
    end

    if q(2)>110*pi/180
        q(2)=q(2)-2*pi;
    end
    if q(2)<-110*pi/180
        q(2)=q(2)+2*pi;
    end

    if q(3)>70*pi/180
        q(3)=q(3)-2*pi;
    end
    if q(3)<-110*pi/180
        q(3)=q(3)+2*pi;
    end

    if q(4)>160*pi/180
        q(4)=q(4)-2*pi;
    end
    if q(4)<-160*pi/180
        q(4)=q(4)+2*pi;
    end

    if q(5)>120*pi/180
        q(5)=q(5)-2*pi;
    end
    if q(5)<-120*pi/180
        q(5)=q(5)+2*pi;
    end

    if q(6)>2*pi
        q(6)=q(6)-2*pi;
    end
    if q(6)<0
        q(6)=q(6)+2*pi;
    end
end