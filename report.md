# Considerazioni finali

## Valori parametri

**distanceThreashold**: 0.2 
la scelta non è ricaduta su 0.1 per avere un frame rate maggiore ma si ha comunque un buon dettaglio di ground, tantè che gli ostacoli non vengono tagliati da un ground troppo alto cosa che accade con parametri leggermente più elevati. 

**clusterTollerance**: 0.3 
è un buon compromesso tra avere un buon dettaglio di cluster ma non troppo fine da avere falsi negativi. Con una tollerance inferiore quello che si ha è molti falsi negativi, molti cluster non vengono identificati mentre con un parametro più alto molti cluster di piccole dimensioni vengono "inglobati" in cluster di dimensione maggiore.

**PERCENT_GROUND** : 0.5


## Dataset 1
Per quel che riguarda il primo dataset gli oggetti della detection con eucliden clustering sono molto "puliti" e non sono presenti falsi positivi, il frame rate è decisamente elevato, infatti ogni pointcloud viene elaborata nell'ordine della decina di ms. Questa considerazione vale per entrambe le implementazioni dell'euclidean clustering, quindi sia usando k-d tree di pcl che l'implementazione che ci è stata fornita. I tempi di esecuzione con il k-d tree che ci è stato fornito sono leggermenti più alti e ciò è dovuto dal fatto che la pcl è una libreria che sfrutta strutture dati e implemetazioni decisamente ottimizzate a livello di linguaggio, ciò che ho fatto per andare a limitare questa problematica è aumentare la leaf size del voxelgrid per ridurre ulteriormente i punti della cloud andando quindi ad elaborare un numero inferiore di punti con i rischi che ne conseguono (ci sono meno dettagli quindi in un contesto differente si possono perdere dei cluster soprattutto se di piccole dimensioni)

## Dataset 2
Per far funzionare il seguente programma con il dataset 2 sono andato ad agire sul parametro della percentuale di punti rimanenti della cloud 
prima dell'interruzione nella fase di segmentazione, portandola al 50% poichè con un valore inferiore parte del ground viene tagliato e non si ha una buona segmentazione, il che portava a falsi negativi. In questo caso le performance sono inferiori rispetto al primo cluster, differenza che è ben evidente andando ad utilizzare la funzione euclideanClustering da me fatta. La ragione che mi sono dato al calo di performance è il numero di cluster che in questo dataset è sensibilmente maggiore il che significa che viene utilizzata più volte la ricerca per raggio, operazione abbastanza complessa infatti andando ad utilizzare la funzione da me implementata dai euclideanClustering all'incirca l'80%del tempo di esecuzione, per ogni frame, è stato speso per la creazione dei cluster (il che denota oltre che l'esecuzione di un operazione complessa una implementazione non ottimizzata).

