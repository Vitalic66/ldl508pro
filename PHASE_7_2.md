# LDL508PRO Phase 7.2

Diese Version aktiviert die bestätigte generische Mehrzielauswertung des ASCII-Modus 0.

## Änderungen

- `HEADA0` bis `HEADA9` werden anhand der Zielanzahl dynamisch dekodiert.
- Die erwartete Telegrammlänge wird als `7 + 4 * Zielanzahl` geprüft.
- Die XOR-Prüfsumme wird vor jeder Zustandsänderung validiert.
- Jedes Distanzfeld muss aus exakt vier ASCII-Ziffern bestehen.
- Mehrere Ziele werden nicht länger als „unbestätigte Datenfelder“ behandelt.
- `target_count`, `max_simultaneous_targets`, `multi_target_active` und
  `multi_target_snapshot` erhalten echte Mehrzielwerte.
- Das bestehende Bestätigungsverfahren für ein einzelnes Ziel bleibt erhalten.
- Die im JSON ausgegebene `id` ist in Phase 7.2 nur der Slot im aktuellen
  Telegramm, keine dauerhafte Fahrzeug-ID.

## Beispiel

`HEADA200310043~` wird als zwei Ziele mit den Entfernungen 31 und 43 dekodiert.

MQTT auf `<topic_prefix>/debug/parsed` beziehungsweise der Textsensor liefert:

```json
{"count":2,"targets":[{"id":0,"r":31.0},{"id":1,"r":43.0}]}
```

## Noch nicht enthalten

Eine stabile Zuordnung derselben Fahrzeuge über mehrere Frames hinweg ist für
Phase 7.3 vorgesehen. Da die Reihenfolge der Felder wechseln kann, darf die
Slot-Nummer noch nicht als Track-ID verwendet werden.
