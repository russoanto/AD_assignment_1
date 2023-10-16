# Considerazioni finali

## Dataset 1
Per quel che riguarda il primo dataset gli oggetti della detection con eucliden clustering sono molto "puliti" e non sono presenti falsi positivi, e il frame rate è decisamente elevato, infatti ogni pointcloud viene elaborata nell'ordine della decina di ms. Questa considerazione vale per entrambe le implementazioni dell'euclidean clustering, quindi sia usando k-d tree di pcl che l'implementazione che ci è stata fornito, ad eccezione dei tempi di esecuzione che sono più alti di un ordine di grandezza (circa).

## Dataset 2
Per quel che riguarda il secondo dataset si hanno vari problemi tra cui:
 -  performance: è un dataset molto denso

