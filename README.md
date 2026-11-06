# team3082-docs

Documentation for Team 3082's 2026Rebuilt robot code.

## Docs

- [Class diagrams](https://mandysedai.github.io/team3082-docs/CLASS_DIAGRAM.html): Mermaid class diagrams of the project's 55 types, split by area (overview, swerve drive and localization, scoring subsystems, autonomous and trajectories, and utilities). ([source](CLASS_DIAGRAM.md))

- [Swerve drive playground](https://mandysedai.github.io/team3082-docs/swerve_rotate_and_drive_playground.html): An interactive page where you drag sliders to see how each wheel's direction and speed are worked out, using the same math as `SwerveManager.rotateAndDrive()`. ([source](swerve_rotate_and_drive_playground.html))

- [Javadoc](https://mandysedai.github.io/team3082-docs/javadoc/index.html): API reference for every package and class in 2026Rebuilt, generated from the code's comments.

## Regenerating the Javadoc

From the 2026Rebuilt project folder, run (change the paths to match your computer):

```bash
./gradlew javadoc --rerun -I ../team3082-docs/tools/javadoc-init.gradle -DjavadocOut=../team3082-docs/javadoc
```

The init script turns off Javadoc's strict comment checks so a few comment mistakes in the robot code don't stop the build. It doesn't change the robot project.
