## Dynamische Bestimmung einer Route durch die Wohnung mit hoher Abdeckung
### Ablauf
- Rechtecke finden
  - https://github.com/lukasalexanderweber/lir
  - Map in befahrbar/nicht-befahrbar binary teilen
    - Unknown == nicht befahrbar
  - Largest-Internal-Rectangle (LIR) finden
  - LIR als nicht Befahrbar markieren
  - Repeat bis LIR kleiner als Person
- Graph über die (Zentren der) Rechtecke bilden
- Tour durch den Graphen finden (Traveling-Salesman-Problem)
  - https://pypi.org/project/python-tsp/
- Rechteckszentren/Knoten als Waypoints nutzen und abfahren

### Sonstiges
Waypoints können genutz werden um nach untersuchung eines Objekts 
die Suche wieder aufzunehmen