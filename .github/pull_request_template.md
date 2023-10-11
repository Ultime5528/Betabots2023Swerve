Description
-----------
<!--- Fais un résumé de ce que tu as ajouté. -->
Closes #TODO. 

Liste de vérification
---------------------
<!-- Entre les [ ], remplace l'espace par un x lorsque c'est fait. -->
- Writing conventions :
    - [ ] English language
    - [ ] File names use lowercase without spaces
    - [ ] Class names use PascalCase
    - [ ] Functions use camelCase
    - [ ] Variable names use snake_case
    - [ ] Function and command names start with an action verb (get, set, move, start, stop...)
    - [ ] Ports respect the naming convention "subsystem" _ "component type" _ "precision"
    - [ ] Properties respect the naming convention
- Command and subsytem safety :
    - [ ] Commands and subsystems inherit from SafeCommand and SafeSubsystem
    - [ ] No commands or subsytems have a `setName()` method, unless necessary
    - [ ] Commands have a `addRequirements()` method that includes all the subsystems they use
- [ ] J'ai exécuté tout mon code en simulation.
