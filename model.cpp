
#include <OpenSim/OpenSim.h>

using SimTK::Inertia;
using SimTK::Pi;
using SimTK::Vec3;

using OpenSim::Body;
using OpenSim::CoordinateSet;
using OpenSim::DisplayGeometry;
using OpenSim::Model;
using OpenSim::PinJoint;

/**
 * Creates an OpenSim model of a three-legged animal. The animal has only one
 * hind leg, centered mediolaterally.
 *
 * Axis orientations, in rough terms:
 * +x: direction of forward travel.
 * +y: up.
 * +z: out of the screen.
 * */

/** Helper functions. **/
void addCylinderDisplayGeometry(Body * body,
        const double & diam, const double & length)
{
    DisplayGeometry * geom = new DisplayGeometry("cylinder.vtp");
    SimTK::Rotation rot;
    // Rotate the cylinder's symmetry (Y) axis to align with the body's X axis:
    rot.setRotationFromAngleAboutZ(-0.5 * Pi);
    geom->setTransform(
            SimTK::Transform(rot, Vec3(0.5 * length, 0, 0)));

    geom->setScaleFactors(Vec3(0.5 * diam, length, 0.5 * diam));
    body->updDisplayer()->setShowAxes(true);
    body->updDisplayer()->updGeometrySet().adoptAndAppend(geom);
};

void addThighCoordinateLimitForce(Model & model, const std::string & coord)
{
    OpenSim::CoordinateLimitForce * force = new OpenSim::CoordinateLimitForce(
                coord, 90, 1.0E2, -90, 1.0E2, 1.0E1, 2.0, false);
	model.addForce(force);
};

void addShankCoordinateLimitForce(Model & model, const std::string & coord)
{
    OpenSim::CoordinateLimitForce * force = new OpenSim::CoordinateLimitForce(
                coord, 10, 1.0E2, -160, 1.0E2, 1.0E1, 2.0, false);
	model.addForce(force);
};

void addCoordinateActuator(Model & model, const std::string & coord)
{
    OpenSim::CoordinateActuator * act = new OpenSim::CoordinateActuator(coord);
    act->setName(coord + "_actuator");
    act->setMinControl(-1000.0);
    act->setMaxControl(1000.0);
    model.addForce(act);
};

void addContactGeometry(Model & model, double xmeas, Body * body,
        const std::string name)
{
    OpenSim::ContactSphere * contact = new OpenSim::ContactSphere(
            0.05, Vec3(xmeas, 0, 0), *body, name);
    contact->setName(name);
    model.addContactGeometry(contact);
}

int main(int argc, char * argv[])
{
    // Preliminaries.
    // --------------
    Model tripod;
    tripod.setName("tripod");

    // Properties.
    // -----------
    // Lengths, in meters.
    double torsoX = 0.5;
    double torsoY = 0.1;
    double torsoZ = 0.3;
    double thighLength = 0.1;
    double thighDiameter = 0.07;
    double shankLength = 0.1;
    double shankDiameter = 0.05;

    // Masses, in kilograms.
    double torsoMass = 1.0;
    double thighMass = 1.0;
    double shankMass = 1.0;

    // Center of mass, in meters.
    // This really just chooses the origin of our bodies.
    Vec3 torsoCOM(0, 0, 0);
    // We choose the x direction to be the axis of the leg segments, with +x
    // pointing toward the ground.
    Vec3 thighCOM(0.5 * thighLength, 0, 0);
    Vec3 shankCOM(0.5 * shankLength, 0, 0);

    // Moments of inertia, in kilograms-meters^2.
    // These are about the center of mass of the body, expressed in the frame
    // of the body.
    Inertia torsoInertia = Inertia::brick(
            0.5 * torsoX, 0.5 * torsoY, 0.5 * torsoZ);
    Inertia thighInertia = Inertia::cylinderAlongX(0.5 * thighDiameter,
            thighLength);
    Inertia shankInertia = Inertia::cylinderAlongX(0.5 * shankDiameter,
            shankLength);

    // Bodies.
    // -------
    Body * torso = new Body("torso",
            torsoMass, torsoCOM, torsoInertia);
    Body * hindThigh = new Body("hind_thigh",
            thighMass, thighCOM, thighInertia);
    Body * hindShank = new Body("hind_shank",
            shankMass, shankCOM, shankInertia);
    Body * frontLeftThigh = new Body("front_left_thigh",
            thighMass, thighCOM, thighInertia);
    Body * frontLeftShank = new Body("front_left_shank",
            shankMass, shankCOM, shankInertia);
    Body * frontRightThigh = new Body("front_right_thigh",
            thighMass, thighCOM, thighInertia);
    Body * frontRightShank = new Body("front_right_shank",
            shankMass, shankCOM, shankInertia);

    // Joints.
    // -------
    Body & ground = tripod.getGroundBody();

    // Ground -> Torso.
    // ````````````````
    // The torso has no constraints with respect to the ground.
    // By default, the tripod's feet are on the ground.
    Vec3 locGTinG(0, thighLength + shankLength, 0);
    Vec3 orientGTinG(0, 0, 0);
    Vec3 locGTinT(0, 0, 0);
    Vec3 orientGTinT(0, 0, 0);
    OpenSim::FreeJoint * groundTorso = new OpenSim::FreeJoint("torso",
            ground, locGTinG, orientGTinG, *torso, locGTinT, orientGTinT);

    CoordinateSet & groundTorsoCS = groundTorso->upd_CoordinateSet();

    // Rotation about x.
    groundTorsoCS[0].setName("torso_rx"); 
    double groundTorsoCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    groundTorsoCS[0].setRange(groundTorsoCS0range);

    groundTorsoCS[1].setName("torso_ry"); 
    double groundTorsoCS1range[2] = {-0.5 * Pi, 0.5 * Pi};
    groundTorsoCS[1].setRange(groundTorsoCS1range);

    groundTorsoCS[2].setName("torso_rz"); 
    double groundTorsoCS2range[2] = {-0.5 * Pi, 0.5 * Pi};
    groundTorsoCS[2].setRange(groundTorsoCS2range);

    // Translation in x.
    groundTorsoCS[3].setName("torso_tx"); 
    double groundTorsoCS3range[2] = {-1.0, 10.0};
    groundTorsoCS[3].setRange(groundTorsoCS3range);

    groundTorsoCS[4].setName("torso_ty"); 
    double groundTorsoCS4range[2] = {0.0, 2.0};
    groundTorsoCS[4].setRange(groundTorsoCS4range);

    groundTorsoCS[5].setName("torso_tz"); 
    double groundTorsoCS5range[2] = {-1.0, 1.0};
    groundTorsoCS[5].setRange(groundTorsoCS5range);

    // Torso -> hind thigh.
    // ````````````````````
    Vec3 locTHinT(-0.5 * torsoX, 0, 0);
    // By default, the leg is extended.
    Vec3 orientTHinT(0, 0, -0.5 * Pi);
    Vec3 locTHinH(0, 0, 0);
    Vec3 orientTHinH(0, 0, 0);
    PinJoint * torsoHindThigh = new PinJoint("hind_thigh",
            *torso, locTHinT, orientTHinT, *hindThigh, locTHinH, orientTHinH);

    CoordinateSet & torsoHindThighCS = torsoHindThigh->upd_CoordinateSet();
    torsoHindThighCS[0].setName("hind_thigh_flexion");
    double torsoHindThighCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    torsoHindThighCS[0].setRange(torsoHindThighCS0range);

    // Hind thigh -> hind shank.
    // `````````````````````````
    Vec3 locHTSinT(thighLength, 0, 0);
    Vec3 orientHTSinT(0, 0, 0);
    Vec3 locHTSinS(0, 0, 0);
    Vec3 orientHTSinS(0, 0, 0);
    PinJoint * hindThighShank = new PinJoint("hind_shank",
            *hindThigh, locHTSinT, orientHTSinT,
            *hindShank, locHTSinS, orientHTSinS);

    CoordinateSet & hindThighShankCS = hindThighShank->upd_CoordinateSet();
    hindThighShankCS[0].setName("hind_knee_extension");
    double hindThighShankCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    hindThighShankCS[0].setRange(hindThighShankCS0range);

    // Torso -> front left thigh.
    // ``````````````````````````
    Vec3 locTFLinT(0.5 * torsoX, 0, -0.5 * torsoZ);
    // By default, the leg is extended.
    Vec3 orientTFLinT(0, 0, -0.5 * Pi);
    Vec3 locTFLinFL(0, 0, 0);
    Vec3 orientTFLinFL(0, 0, 0);
    PinJoint * torsoFrontLeftThigh = new PinJoint("front_left_thigh",
            *torso, locTFLinT, orientTFLinT,
            *frontLeftThigh, locTFLinFL, orientTFLinFL);

    CoordinateSet & torsoFrontLeftThighCS =
        torsoFrontLeftThigh->upd_CoordinateSet();
    torsoFrontLeftThighCS[0].setName("front_left_thigh_flexion");
    double torsoFrontLeftThighCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    torsoFrontLeftThighCS[0].setRange(torsoFrontLeftThighCS0range);

    // Front left thigh -> front left shank.
    // `````````````````````````````````````
    Vec3 locFLTSinT(thighLength, 0, 0);
    Vec3 orientFLTSinT(0, 0, 0);
    Vec3 locFLTSinS(0, 0, 0);
    Vec3 orientFLTSinS(0, 0, 0);
    PinJoint * frontLeftThighShank = new PinJoint("front_left_shank",
            *frontLeftThigh, locFLTSinT, orientFLTSinT,
            *frontLeftShank, locFLTSinS, orientFLTSinS);

    CoordinateSet & frontLeftThighShankCS =
        frontLeftThighShank->upd_CoordinateSet();
    frontLeftThighShankCS[0].setName("front_left_knee_extension");
    double frontLeftThighShankCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    frontLeftThighShankCS[0].setRange(frontLeftThighShankCS0range);

    // Torso -> front right thigh.
    // ```````````````````````````
    Vec3 locTFRinT(0.5 * torsoX, 0, 0.5 * torsoZ);
    // By default, the leg is extended.
    Vec3 orientTFRinT(0, 0, -0.5 * Pi);
    Vec3 locTFRinFR(0, 0, 0);
    Vec3 orientTFRinFR(0, 0, 0);
    PinJoint * torsoFrontRightThigh = new PinJoint("front_right_thigh",
            *torso, locTFRinT, orientTFRinT,
            *frontRightThigh, locTFRinFR, orientTFRinFR);

    CoordinateSet & torsoFrontRightThighCS =
        torsoFrontRightThigh->upd_CoordinateSet();
    torsoFrontRightThighCS[0].setName("front_right_thigh_flexion");
    double torsoFrontRightThighCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    torsoFrontRightThighCS[0].setRange(torsoFrontRightThighCS0range);

    // Front right thigh -> front right shank.
    // ```````````````````````````````````````
    Vec3 locFRTSinT(thighLength, 0, 0);
    Vec3 orientFRTSinT(0, 0, 0);
    Vec3 locFRTSinS(0, 0, 0);
    Vec3 orientFRTSinS(0, 0, 0);
    PinJoint * frontRightThighShank = new PinJoint("front_right_shank",
            *frontRightThigh, locFRTSinT, orientFRTSinT,
            *frontRightShank, locFRTSinS, orientFRTSinS);

    CoordinateSet & frontRightThighShankCS =
        frontRightThighShank->upd_CoordinateSet();
    frontRightThighShankCS[0].setName("front_right_knee_extension");
    double frontRightThighShankCS0range[2] = {-0.5 * Pi, 0.5 * Pi};
    frontRightThighShankCS[0].setRange(frontRightThighShankCS0range);

    // Add bodies to the model.
    // ------------------------
    // Now that we've define the joints.
    tripod.addBody(torso);
    tripod.addBody(hindThigh);
    tripod.addBody(hindShank);
    tripod.addBody(frontLeftThigh);
    tripod.addBody(frontLeftShank);
    tripod.addBody(frontRightThigh);
    tripod.addBody(frontRightShank);

    // Display geometry.
    // -----------------

    // Torso.
    // ``````
    DisplayGeometry * torsoDisplay = new DisplayGeometry("box.vtp");
    torsoDisplay->setScaleFactors(
            Vec3(torsoX, torsoY,  torsoZ));
    torso->updDisplayer()->updGeometrySet().adoptAndAppend(torsoDisplay);
    torso->updDisplayer()->setShowAxes(true);

    // Limbs.
    // ``````
    addCylinderDisplayGeometry(hindThigh, thighDiameter, thighLength);
    addCylinderDisplayGeometry(hindShank, shankDiameter, shankLength);
    addCylinderDisplayGeometry(frontLeftThigh, thighDiameter, thighLength);
    addCylinderDisplayGeometry(frontLeftShank, shankDiameter, shankLength);
    addCylinderDisplayGeometry(frontRightThigh, thighDiameter, thighLength);
    addCylinderDisplayGeometry(frontRightShank, shankDiameter, shankLength);

    // Enforce joint limits on the legs.
    // ---------------------------------
    addThighCoordinateLimitForce(tripod, "hind_thigh_flexion");
    addThighCoordinateLimitForce(tripod, "front_left_thigh_flexion");
    addThighCoordinateLimitForce(tripod, "front_right_thigh_flexion");
    addShankCoordinateLimitForce(tripod, "hind_knee_extension");
    addShankCoordinateLimitForce(tripod, "front_left_knee_extension");
    addShankCoordinateLimitForce(tripod, "front_right_knee_extension");

    // Actuators.
    // ----------
    addCoordinateActuator(tripod, "hind_thigh_flexion");
    addCoordinateActuator(tripod, "front_left_thigh_flexion");
    addCoordinateActuator(tripod, "front_right_thigh_flexion");
    addCoordinateActuator(tripod, "hind_knee_extension");
    addCoordinateActuator(tripod, "front_left_knee_extension");
    addCoordinateActuator(tripod, "front_right_knee_extension");

    // Contact.
    // --------
    // So the tripod does not go through the ground.
    OpenSim::ContactHalfSpace * groundContact = new OpenSim::ContactHalfSpace(
            Vec3(0), Vec3(0.0, 0.0, -0.5 * Pi), ground);
    groundContact->setName("ground_contact");
    tripod.addContactGeometry(groundContact);

    // Limbs.
    // ``````
    addContactGeometry(tripod, shankLength, hindShank, "hind_foot_contact");
    addContactGeometry(tripod, shankLength, frontLeftShank,
            "front_left_foot_contact");
    addContactGeometry(tripod, shankLength, frontRightShank,
            "front_right_foot_contact");

    // Print the model.
    tripod.print("tripod.osim");

    return EXIT_SUCCESS;
}

