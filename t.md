La funzione atan2(y, x) è una variante della normale arcotangente ($\arctan$) che calcola l'angolo in radianti tra l'asse positivo delle ascisse e il punto (x, y) in un piano cartesiano.
A differenza della classica $\arctan(y/x)$, la funzione atan2 prende i due argomenti separatamente per capire in quale dei 4 quadranti si trova il punto e gestisce automaticamente il caso in cui x = 0 (evitando la divisione per zero).
## La formula definita per casi
La formula matematica di atan2(y, x) è definita nel seguente modo:
$$\text{atan2}(y, x) = \begin{cases} \arctan\left(\frac{y}{x}\right) & \text{se } x > 0 \\ \arctan\left(\frac{y}{x}\right) + \pi & \text{se } x < 0 \text{ e } y \ge 0 \\ \arctan\left(\frac{y}{x}\right) - \pi & \text{se } x < 0 \text{ e } y < 0 \\ +\frac{\pi}{2} & \text{se } x = 0 \text{ e } y > 0 \\ -\frac{\pi}{2} & \text{se } x = 0 \text{ e } y < 0 \\ \text{non definito} & \text{se } x = 0 \text{ e } y = 0 \end{cases} \end{cases}$$ 
Nota: Nei linguaggi di programmazione, se x=0 e y=0, di solito la funzione restituisce 0 invece di dare errore.
------------------------------
## Perché si usa al posto di $\arctan(y/x)$?
Se provi a calcolare l'angolo usando solo il rapporto $\frac{y}{x}$, perdi l'informazione sui segni originari.
Ad esempio:

* Se il punto è nel 1° quadrante: $(x=3, y=3) \rightarrow \frac{3}{3} = 1 \rightarrow \arctan(1) = 45^\circ$
* Se il punto è nel 3° quadrante: $(x=-3, y=-3) \rightarrow \frac{-3}{-3} = 1 \rightarrow \arctan(1) = 45^\circ$ (Sbagliato! L'angolo reale è -135° o 225°)

La funzione atan2 guarda i segni di x e y prima di fare la divisione, restituendo sempre l'angolo corretto compreso nell'intervallo $[-\pi, \pi]$ (ovvero tra -180° e +180°).
Ti serve aiuto per implementare questa formula in un linguaggio di programmazione specifico (come C, C++ o Matlab), oppure vuoi vedere come applicarla ai valori del tuo accelerometro?

