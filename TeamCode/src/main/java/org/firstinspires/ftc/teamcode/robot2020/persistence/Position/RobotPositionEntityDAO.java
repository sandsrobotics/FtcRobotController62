package org.firstinspires.ftc.teamcode.robot2020.persistence.Position;

import androidx.room.Dao;
import androidx.room.Insert;
import androidx.room.Query;

import org.firstinspires.ftc.teamcode.robot2020.Movement;
import org.firstinspires.ftc.teamcode.robot2020.persistence.Movement.MovementEntity;

import java.util.List;

@Dao
public interface RobotPositionEntityDAO {
    @Insert
    void insertAll(RobotPositionEntity... entities);

    @Query("SELECT * FROM Position")
    List<RobotPositionEntity> getAll();

    @Query("SELECT * FROM Position WHERE `run number` = :runNum")
    List<RobotPositionEntity> getRun(int runNum);

    @Query("SELECT * FROM Position WHERE id = (select max(id) from position)")
    RobotPositionEntity getLastByID();

    @Query("SELECT * FROM Position WHERE `time of entry` = (select max(`time of entry`) from Position)")
    RobotPositionEntity getLastByTime();

    @Query("Select max(`run number`) from Position")
    int getLastRunNum();

    @Query("SELECT * FROM Position WHERE `run number` = (Select max(`run number`) from Position)")
    List<RobotPositionEntity> getLastRun();

    @Query("DELETE FROM Position")
    void deleteAll();
}
