# Kód do předmětu BPC-PRP
Šablona převzata z: [Robotics-BUT](https://github.com/Robotics-BUT/bpc-prp-cpp-project-template/tree/53c2f887439a6ac508bb4e62157084f8cd38cab8)

Organizace složek se zdroji:
- algorithms - použité algoritmy (PID, detekce tagů,...)
- loops - řídící smyčky
- nodes - ROS2 nody pro jednotlivé zařízení na robotovi

Projekt neobsahuje launch file, jak je běžné pro ROS2 projekty, všechny nody se spouští v main().

Spuštění pomocí příkazu:
`ros2 run prp_project main`

Projekt využívá frameworku ROS2 (verze Humble), před prací s tímto projektem je vřele doporučeno se s frameworkem seznámit [ROS2](https://docs.ros.org/en/humble/index.html)

Autoři:
- Jan Drochýtek (256278)
- Marek Holečka (256294)
