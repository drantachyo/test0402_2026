package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import java.util.Map;
import java.util.TreeMap;

public class ShotCalculator {

    // Таблица: Дистанция (дюймы) -> Позиция Серво (0.0 - 1.0)
    // TreeMap автоматически сортирует ключи по возрастанию
    private final TreeMap<Double, Double> calibrationTable = new TreeMap<>();

    public ShotCalculator() {
        // =========================================================
        // СЮДА ВПИСЫВАЙ ДАННЫЕ ИЗ ТВОЕГО ЛИСТОЧКА (InterpolationTuner)
        // =========================================================
        // Пример (замени на свои реальные цифры):

        addCalibration(0,    0.32); // В упор// Далеко

        // Чем больше точек, тем точнее стрельба!
    }

    // Метод для добавления точки калибровки
    public void addCalibration(double distanceInches, double servoPos) {
        calibrationTable.put(distanceInches, servoPos);
    }

    /**
     * ГЛАВНЫЙ МЕТОД:
     * Принимает дистанцию (с камеры или одометрии) и выдает позицию сервы.
     */
    public double calculateHoodPos(double distance) {
        return getInterpolatedValue(distance);
    }

    // Математика линейной интерполяции
    private double getInterpolatedValue(double dist) {
        // 1. Если таблица пустая — возвращаем дефолт
        if (calibrationTable.isEmpty()) return 0.35;

        // 2. Ищем ближайшие точки "снизу" и "сверху"
        Map.Entry<Double, Double> floor = calibrationTable.floorEntry(dist);
        Map.Entry<Double, Double> ceiling = calibrationTable.ceilingEntry(dist);

        // 3. Обработка краев (если дистанция меньше минимума или больше максимума)
        if (floor == null) return ceiling.getValue(); // Меньше самой ближней точки
        if (ceiling == null) return floor.getValue(); // Больше самой дальней точки

        // 4. Если попали точно в точку
        if (floor.getKey().equals(ceiling.getKey())) return floor.getValue();

        // 5. ЛИНЕЙНАЯ ИНТЕРПОЛЯЦИЯ
        // Формула: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double x = dist;
        double x1 = floor.getKey(); // Дистанция 1
        double x2 = ceiling.getKey(); // Дистанция 2
        double y1 = floor.getValue(); // Худ 1
        double y2 = ceiling.getValue(); // Худ 2

        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}