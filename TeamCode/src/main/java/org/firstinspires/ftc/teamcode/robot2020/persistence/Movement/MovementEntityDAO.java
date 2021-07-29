package org.firstinspires.ftc.teamcode.robot2020.persistence.Movement;

import androidx.room.Dao;
import androidx.room.Insert;
import androidx.room.Query;

import java.util.List;

@Dao
public interface MovementEntityDAO {

    @Insert
    void insertAll(MovementEntity... entities);

    @Query("SELECT * FROM Movement")
    List<MovementEntity> getAll();

    @Query("SELECT * FROM MOVEMENT WHERE name = :name ORDER BY ID")
    List<MovementEntity> loadMovementByName(String name);

    @Query("DELETE FROM MOVEMENT")
    void deleteAll();
}
