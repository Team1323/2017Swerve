package com.team1323.frc2017.paths;

import java.util.ArrayList;

import com.team1323.frc2017.paths.PathBuilder.Waypoint;
import com.team254.lib.util.control.Path;

import Utilities.RigidTransform2d;
import Utilities.Rotation2d;
import Utilities.Translation2d;

public class StartToBoilerGearBlue implements PathContainer{
	@Override
    public Path buildPath() {
		RigidTransform2d start = new RigidTransform2d();
		RigidTransform2d end = new RigidTransform2d(new Translation2d(96.0, -24.0), Rotation2d.fromDegrees(-60));
		Translation2d middle = start.intersection(end);
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(start.getTranslation(), 0, 0));
        sWaypoints.add(new Waypoint(middle, 45, 60));
        sWaypoints.add(new Waypoint(end.getTranslation(), 0, 60));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
