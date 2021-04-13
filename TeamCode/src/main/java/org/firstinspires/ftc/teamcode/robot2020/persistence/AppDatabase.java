package org.firstinspires.ftc.teamcode.robot2020.persistence;

import androidx.room.Database;
import androidx.room.RoomDatabase;

import org.firstinspires.ftc.teamcode.robot2020.persistence.Movement.MovementEntity;
import org.firstinspires.ftc.teamcode.robot2020.persistence.Movement.MovementEntityDAO;
import org.firstinspires.ftc.teamcode.robot2020.persistence.Position.RobotPositionEntity;
import org.firstinspires.ftc.teamcode.robot2020.persistence.Position.RobotPositionEntityDAO;

@Database(entities = {MovementEntity.class}, version = 3, exportSchema = false)
public abstract class AppDatabase extends RoomDatabase {

    public abstract MovementEntityDAO movementEntityDAO();

}
