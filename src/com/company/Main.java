package com.company;
import gurobi.*;
import jdk.swing.interop.SwingInterOpUtils;

import java.util.ArrayList;

public class Main {
    /** LINEARIZZAZIONE MODULO:
     * min |a-b|  ---->   min w
     *                      w >= a-b
     *                      w >= b-a
     *
     * a= sommatoria xij, i=1...M, j=1...K/2
     * b= sommatoria xij, i=1...M, j=(k/2)+1.....K
     *
     *
     */

    private static int numSlack=0;
    private static int numAus=0;
    //creazione variabili xij del modello
    private static GRBVar[][] aggiungiVariabili(GRBModel model, int num_emittenti, int num_fasceOrarie) throws GRBException {
        GRBVar[][] xij = new GRBVar[num_emittenti][num_fasceOrarie];

        for (int i = 0; i < num_emittenti; i++){

           for (int j = 0; j < num_fasceOrarie; j++){
                    xij[i][j] = model.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, "x"+i+j);
           }
        }
            return xij;
    }

    //creazione variabile per funzione obiettivo linearizzata (modulo linearizzato)
    private static GRBVar aggiungiVariabileFunzioneObiettivo(GRBModel model) throws GRBException {
        GRBVar W;
        W= model.addVar( -GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "W");
            return W;
    }

    //creazione variabili di slack/surplus
    private static GRBVar[] aggiungiVariabili(GRBModel model, int numVar) throws GRBException {
        GRBVar[] yh = new GRBVar[numVar];

        for (int i = 0; i < numVar; i++){
                yh[i] = model.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, "yh"+i);
        }

        return yh;
    }

    //aggiunta funzione obiettivo al modello (modulo linearizzato)
    private static void aggiungiFunzioneObiettivo(GRBModel model, GRBVar w) throws GRBException {
        GRBLinExpr funzione_obiettivo = new GRBLinExpr();
        // W = |a-b|
        //**** AGGIUNTA FUNZIONE OBIETTIVO ****
        funzione_obiettivo.addTerm(1.0, w);
        model.setObjective(funzione_obiettivo);
        model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);
    }

    //aggiunta funzione obiettivo al modello ausiliario per <prima fase> del Metodo due fasi (modulo linearizzato)
    private static void aggiungiFunzioneObiettivoAus(GRBModel model, GRBVar[] aus) throws GRBException {
        GRBLinExpr funzione_obiettivo = new GRBLinExpr();
        for(int i=0; i<aus.length; i++){
            funzione_obiettivo.addTerm(1.0, aus[i]);
        }
        model.setObjective(funzione_obiettivo);
        model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);
    }

    // aggiunta vincoli per modulo linearizzato, modello ausiliario
    private static void aggiungiVincoliFunzioneObiettivoAus(GRBModel model, GRBVar w, GRBVar[][] xij, int[][] P, GRBVar[] yh, GRBVar[] aus) throws GRBException {
        // W = |a-b|
        GRBLinExpr obj = new GRBLinExpr();    // a-b
        GRBLinExpr obj1 = new GRBLinExpr();    // b-a

        for (int i = 0; i < P.length; i++) {
            for (int j = 0; j < P[0].length / 2; j++) {
                obj.addTerm(P[i][j], xij[i][j]);
                obj1.addTerm((-1) * P[i][j], xij[i][j]);
            }

            for (int j = (P[0].length / 2) + 1; j < P[0].length; j++) {
                obj.addTerm((-1) * (P[i][j]), xij[i][j]);
                obj1.addTerm((P[i][j]), xij[i][j]);
            }
        }

        obj.addTerm(1, w);    // w > b-a, w-b+a > 0
        obj1.addTerm(1, w);    // w > a-b, w-a+b > 0

        obj.addTerm(-1, yh[numSlack]);
        numSlack++;
        obj.addTerm(-1, yh[numAus]);
        numAus++;
        model.addConstr(obj, GRB.EQUAL, 0, "vincolo forma standard w>=b-a");

        obj1.addTerm(-1, yh[numSlack]);
        numSlack++;
        obj1.addTerm(-1, yh[numAus]);
        numAus++;
        model.addConstr(obj1, GRB.EQUAL, 0, "vincolo forma standard w>=a-b");
    }

    // aggiunta vincoli per modulo linearizzato, modello linearizzato
    private static void aggiungiVincoliFunzioneObiettivo(GRBModel model, GRBVar w, GRBVar[][] xij, int[][] P, GRBVar[] yh) throws GRBException {
        // W = |a-b|
        GRBLinExpr obj = new GRBLinExpr();	// a-b
        GRBLinExpr obj1 = new GRBLinExpr();	// b-a

        for (int i = 0; i < P.length; i++){
            for (int j = 0; j < P[0].length/2; j++){
                obj.addTerm(P[i][j], xij[i][j]);
                obj1.addTerm((-1)*P[i][j], xij[i][j]);
            }

            for (int j = (P[0].length/2)+1; j < P[0].length; j++){
                obj.addTerm((-1)*(P[i][j]), xij[i][j]);
                obj1.addTerm((P[i][j]), xij[i][j]);
            }
        }
        obj.addTerm(1, w);	// w > b-a, w-b+a > 0
        obj1.addTerm(1, w);	// w > a-b, w-a+b > 0

        //********** VINCOLO FORMA STANDARD **********
        obj.addTerm(-1, yh[numSlack]);
        numSlack++;
        model.addConstr(obj, GRB.EQUAL, 0, "vincolo forma standard w>=b-a");
        obj1.addTerm(-1, yh[numSlack]);
        numSlack++;
        model.addConstr(obj1, GRB.EQUAL, 0, "vincolo forma standard w>=a-b");
        //*********************************************

        //model.addConstr(obj, GRB.GREATER_EQUAL, 0, "vincolo w>=b-a");
        //model.addConstr(obj1, GRB.GREATER_EQUAL, 0, "vincolo w>=a-b");
    }

    //aggiunta vincoli: l'emittente non pu`o acquistare più di τij minuti
    private static void aggiungiVincoloTempoAcq(GRBModel model, GRBVar[][] xij,int[][] tau, GRBVar[] yh) throws GRBException {

        for (int i = 0; i < xij.length; i++){
            for (int j = 0; j < xij[0].length; j++){
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1, xij[i][j]);

                // ********** VINCOLO FORMA STANDARD **********
                expr.addTerm(1, yh[numSlack]);
                numSlack++;
                model.addConstr(expr, GRB.EQUAL, tau[i][j], "vincolo tempo acquistabile" + i+j);
                // ********************************************

                //model.addConstr(expr, GRB.LESS_EQUAL, tau[i][j], "vincolo tempo acquistabile" + i+j);a
            }
        }
    }

    //aggiunto vincoli: La direzione non investe più di βi euro su una qualsivoglia emittente i
    private static void aggiungiVincoloInvestimnetosuEmittente(GRBModel model, GRBVar[][] xij, int[][] C, int[] beta, GRBVar[] yh) throws GRBException{

        for (int i = 0; i < xij.length; i++){
            GRBLinExpr expr = new GRBLinExpr();

            for (int j = 0; j < xij[0].length; j++){
                expr.addTerm(C[i][j], xij[i][j]);
            }

            // ********** VINCOLO FORMA STANDARD **********
            expr.addTerm(1, yh[numSlack]);
            numSlack++;
            model.addConstr(expr, GRB.EQUAL, beta[i], "vincolo investimento su emittente" + i);
            // ********************************************

            //model.addConstr(expr, GRB.LESS_EQUAL, beta[i], "vincolo investimento su emittente" + i);
        }
    }

    //aggiunto vincoli: La direzione vuole investire almeno Ω% del budget totale su ogni fascia oraria j
    private static void aggiungiVincoloMinBudgetSuFascia(GRBModel model, GRBVar[][] xij, int [][] C, int omega, int budget_tot, GRBVar[] yh) throws GRBException{
        for (int j = 0; j < xij[0].length; j++){
            GRBLinExpr expr = new GRBLinExpr();

            for (int i = 0; i < xij.length; i++) {
                expr.addTerm(C[i][j], xij[i][j]);
            }

            // ********** VINCOLO FORMA STANDARD **********
            expr.addTerm(-1, yh[numSlack]);
            numSlack++;
            model.addConstr(expr, GRB.EQUAL, (omega/100)*budget_tot, "vincolo swl minimo budget sulla fascia oraria" + j);
            // ********************************************

            //model.addConstr(expr, GRB.GREATER_EQUAL, (omega/100)*budget_tot, "vincolo budget per fascia" + j);
        }
    }

    //aggiunto vincoli: l’azienda desidera ottenere una copertura giornaliera complessiva di almeno S spettatori
    private static void aggiungiVincolospettatori(GRBModel model, GRBVar[][] xij, int [][] P, int S, GRBVar[] yh) throws GRBException {
        GRBLinExpr expr = new GRBLinExpr();

        for (int i = 0; i < xij.length; i++){
            for (int j = 0; j < xij[0].length; j++){
                expr.addTerm(P[i][j], xij[i][j]);
            }
        }

        // ********** VINCOLO FORMA STANDARD **********
        expr.addTerm(-1, yh[numSlack]);
        numSlack++;
        model.addConstr(expr, GRB.EQUAL, S, "vincolo spettatori ");
        // ********************************************

        //model.addConstr(expr, GRB.GREATER_EQUAL, S, "vincolo spettatori ");
        }

    // arrotondare i numeri double a n cifre decimali
    private static double arrotondamento(double val,int numCifreDecimali) {
        double t = Math.pow(10, numCifreDecimali);
        return Math.round(val*t)/t;
    }

    //aggiunta vincoli: l'emittente non pu`o acquistare più di τij minuti [Modello Ausiliario]
    private static void aggiungiVincoloTempoAcqAus(GRBModel model, GRBVar[][] xij,int[][] tau, GRBVar[] yh, GRBVar[] aus) throws GRBException {
        for (int i = 0; i < xij.length; i++) {
            for (int j = 0; j < xij[0].length; j++) {
                GRBLinExpr expr = new GRBLinExpr();
                expr.addTerm(1, xij[i][j]);

                expr.addTerm(1, yh[numSlack]);
                numSlack++;
                expr.addTerm(1, aus[numAus]);
                numAus++;
                model.addConstr(expr, GRB.EQUAL, tau[i][j], "vincolo tempo acquistabile" + i + j);

            }
        }
    }

    //aggiunto vincoli: La direzione non investe più di βi euro su una qualsivoglia emittente i [Modello Ausiliario]
    private static void aggiungiVincoloInvestimnetosuEmittenteAus (GRBModel model, GRBVar[][]xij,int[][] C, int[] beta, GRBVar[] yh, GRBVar[]aus) throws GRBException {

        for (int i = 0; i < xij.length; i++) {
            GRBLinExpr expr = new GRBLinExpr();

            for (int j = 0; j < xij[0].length; j++) {
                expr.addTerm(C[i][j], xij[i][j]);
            }

            expr.addTerm(1, yh[numSlack]);
            numSlack++;
            expr.addTerm(1, aus[numAus]);
            numAus++;
            model.addConstr(expr, GRB.EQUAL, beta[i], "vincolo investimento su emittente" + i);

        }
    }

    //aggiunto vincoli: La direzione vuole investire almeno Ω% del budget totale su ogni fascia oraria j [Modello Ausiliario]
    private static void aggiungiVincoloMinBudgetSuFasciaAus (GRBModel model, GRBVar[][]xij,int[][] C, int omega,
                                                             int budget_tot, GRBVar[] yh, GRBVar[]aus) throws GRBException {
        for (int j = 0; j < xij[0].length; j++) {
            GRBLinExpr expr = new GRBLinExpr();

            for (int i = 0; i < xij.length; i++) {
                expr.addTerm(C[i][j], xij[i][j]);
            }

            expr.addTerm(-1, yh[numSlack]);
            numSlack++;
            expr.addTerm(1, aus[numAus]);
            numAus++;
            model.addConstr(expr, GRB.EQUAL, (omega / 100) * budget_tot, "vincolo swl minimo budget sulla fascia oraria" + j);

        }
    }

    //aggiunto vincoli: l’azienda desidera ottenere una copertura giornaliera complessiva di almeno S spettatori [Modello Ausiliario]
    private static void aggiungiVincolospettatoriAus (GRBModel model, GRBVar[][]xij,int[][] P,
                                                      int S, GRBVar[] yh, GRBVar[]aus) throws GRBException {
        GRBLinExpr expr = new GRBLinExpr();

        for (int i = 0; i < xij.length; i++) {
            for (int j = 0; j < xij[0].length; j++) {
                expr.addTerm(P[i][j], xij[i][j]);
            }
        }
        expr.addTerm(-1, yh[numSlack]);
        numSlack++;
        expr.addTerm(1, aus[numAus]);
        numAus++;
        model.addConstr(expr, GRB.EQUAL, S, "vincolo spettatori ");

    }

    public static void main (String[]args){
          final double EPSILON = 1E-5;
          int M = 10;
          int K = 8;
          int S = 82215;
          int omega = 2;
          int[] beta = {2953, 3314, 2904, 3487, 3222, 3492, 3455, 3388, 2595, 2806};
          int budget_tot = 0;
          int numVarSlack = 0;

          int[][] tau = {{3, 3, 2, 3, 2, 3, 2, 1},
                      {2, 2, 1, 2, 1, 1, 3, 2},
                      {2, 1, 1, 1, 1, 2, 1, 3},
                      {2, 3, 2, 1, 2, 2, 1, 3},
                      {2, 1, 1, 2, 2, 2, 2, 2},
                      {2, 1, 3, 1, 1, 3, 2, 3},
                      {2, 2, 3, 3, 1, 1, 1, 2},
                      {2, 1, 2, 1, 3, 2, 2, 1},
                      {2, 3, 3, 3, 2, 1, 1, 1},
                      {2, 1, 3, 3, 2, 2, 2, 3}};

          int[][] C = {
                  {1092, 1066, 1166, 1310, 1151, 1138, 1021, 1005},
                  {1138, 1049, 1008, 1287, 1220, 1076, 1240, 1228},
                  {927, 1100, 1026, 998, 1251, 1286, 953, 1287},
                  {1104, 1111, 992, 1260, 1168, 1332, 1006, 1021},
                  {1364, 1280, 1089, 1012, 1214, 1340, 1193, 1376},
                  {1230, 1348, 1104, 901, 1009, 1092, 1224, 918},
                  {1231, 1066, 1294, 915, 972, 1396, 908, 901},
                  {1097, 986, 1094, 1261, 1051, 1052, 984, 1396},
                  {1363, 968, 911, 1003, 980, 1113, 1395, 1397},
                  {918, 1376, 1178, 1252, 1159, 1211, 1011, 1153}
                };

          int[][] P = {{565, 2894, 1105, 936, 1178, 2564, 3085, 1204},
                    {392, 1714, 2850, 3406, 2971, 3384, 450, 736},
                    {972, 1363, 585, 3049, 1262, 2811, 2151, 666},
                    {3318, 2139, 2606, 354, 2439, 706, 667, 2345},
                    {2820, 2263, 495, 1957, 872, 2073, 3246, 2089},
                    {3344, 1073, 697, 813, 1936, 811, 2562, 2227},
                    {497, 580, 1577, 1492, 3214, 2272, 2080, 573},
                    {1178, 2159, 3198, 2142, 1899, 1993, 2845, 2001},
                    {1752, 1065, 951, 1284, 3227, 1281, 533, 620},
                    {1661, 2472, 1049, 2223, 1734, 569, 861, 855}
          };
    //-------------- CALCOLO DATI AGGIUNTIVI ----------------------
        for (int i = 0; i < beta.length; i++) {
            budget_tot += beta[i];
        }
        numVarSlack = M + M * K + 1 + 2 + K;
    //-----------------------------------------------------------
        GRBEnv env;
        try {
            // SETTAGGIO MODELLO
            env = new GRBEnv("elaborato1Gurobi.log");
            env.set(GRB.IntParam.Presolve, 0);
            env.set(GRB.IntParam.Method, 0);

                    GRBModel model = new GRBModel(env);
                    GRBModel modelAggiuntivo = new GRBModel(env);

        //-------------- AGGIUNTA VARIABILI -----------------------------------
            GRBVar[][] xij = aggiungiVariabili(model, M, K);
            GRBVar W = aggiungiVariabileFunzioneObiettivo(model);
            GRBVar[] yh = aggiungiVariabili(model, numVarSlack);
        //---------------------------------------------------------------------

        //-------------- FUNZIONE OBIETTIVO -------------------------------------
            aggiungiFunzioneObiettivo(model, W);    //modulo linearizzato
        //------------------------------------------------------------------------

        //-------------------- VINCOLI -----------------------------------------------------
                aggiungiVincoliFunzioneObiettivo(model, W, xij, P, yh);
                aggiungiVincoloTempoAcq(model, xij, tau, yh);
                aggiungiVincoloInvestimnetosuEmittente(model, xij, C, beta, yh);
                aggiungiVincoloMinBudgetSuFascia(model, xij, C, omega, budget_tot, yh);
                aggiungiVincolospettatori(model, xij, P, S, yh);

                model.optimize();    //ottimizzazio
        //-----------------------------------------------------------------------------------

//*****************************************************************************************************************
            //creazione problema ausiliario
                GRBVar[][] xijAus = aggiungiVariabili(modelAggiuntivo, M, K);
                GRBVar WAus = aggiungiVariabileFunzioneObiettivo(modelAggiuntivo);
                GRBVar[] yhAus = aggiungiVariabili(modelAggiuntivo, numVarSlack);
                GRBVar[] aus = aggiungiVariabili(modelAggiuntivo, numVarSlack);    //variabili ausiliarie
                numSlack =0;
                numAus=0;
                aggiungiFunzioneObiettivoAus(modelAggiuntivo,aus);
                aggiungiVincoliFunzioneObiettivoAus(modelAggiuntivo, WAus, xijAus, P, yhAus, aus);
                aggiungiVincoloTempoAcqAus(modelAggiuntivo, xijAus, tau, yhAus, aus);
                aggiungiVincoloInvestimnetosuEmittenteAus(modelAggiuntivo, xijAus, C, beta, yhAus, aus);
                aggiungiVincoloMinBudgetSuFasciaAus(modelAggiuntivo, xijAus, C, omega, budget_tot, yhAus, aus);
                aggiungiVincolospettatoriAus(modelAggiuntivo, xijAus, P, S, yhAus, aus);

                modelAggiuntivo.optimize();
//******************************************************************************************************************
            System.out.println("\nGRUPPO gruppo_25");
            System.out.println("Componenti: Brognoli e Agosti\n\n");
//------------------------------- QUESITO 1 ---------------------------------------------------------------------------
            System.out.println("QUESITO I:");
            System.out.println("funzione obiettivo = " + arrotondamento(model.get(GRB.DoubleAttr.ObjVal), 4));
       //______ TOT SPETTATORI ____________
            double totSpettatori = 0;
            for (int i = 0; i < xij.length; i++) {

                for (int j = 0; j < xij[0].length; j++) {
                    totSpettatori += P[i][j] * xij[i][j].get(GRB.DoubleAttr.X);
                }

            }
            System.out.println("copertura raggiunta totale (spettatori) = " + arrotondamento(totSpettatori, 4));    //o anche (S+yh[nSlack-1].get(DoubleAttr.X))
       // _______ TEMPO ACQUISTATO ___________
            double tempoAcqistato = 0;
            for (int i = 0; i < xij.length; i++) {

                for (int j = 0; j < xij[0].length; j++) {
                    tempoAcqistato += xij[i][j].get(GRB.DoubleAttr.X);
                }

            }
            System.out.println("tempo acquistato (minuti) = " + arrotondamento(tempoAcqistato, 4));
       //__________BUDGET UTILIZZATO _________
            double budgetUtilizzato = 0;
            for (int i = 0; i < xij.length; i++) {

                for (int j = 0; j < xij[0].length; j++) {
                    budgetUtilizzato += C[i][j] * xij[i][j].get(GRB.DoubleAttr.X);
                }
            }
            System.out.println("budget inutilizzato = " + arrotondamento(budget_tot - budgetUtilizzato, 4));
       //___________ SOLUZIONE OTTIMA _____________
            System.out.println("soluzione di base ottima:");
            for (GRBVar var : model.getVars()) {
                System.out.println(var.get(GRB.StringAttr.VarName) + " = " + arrotondamento(var.get(GRB.DoubleAttr.X), 4));
            }
//--------------------------------------------------------------------------------------------------------------------

            System.out.println("\n");

//---------------------------- QUESITO 2 -----------------------------------------------------------------------------
            System.out.println("QUESITO II:");

        // VARIABILI IN BASE
            ArrayList<Integer> variabileBase = new ArrayList<>();
            for (GRBVar var : model.getVars()) {
                if (var.get(GRB.IntAttr.VBasis) == 0) {    //se valore di VBasis == 0 la variabile è in base
                    variabileBase.add(1);
                } else {
                    variabileBase.add(0);
                }
            }
            System.out.print("variabili in base: [");
            for (int i = 0; i < variabileBase.size() - 1; i++) {
                System.out.print(variabileBase.get(i) + ", ");
            }
            System.out.println(variabileBase.get(variabileBase.size() - 1) + "]");

        //COEFFICIENTI DI COSTO RIDOTTO
            System.out.print("coefficienti di costo ridotto: [");
            int numVar = model.get(GRB.IntAttr.NumVars);
            for (int i = 0; i < numVar - 1; i++) {
                System.out.print(arrotondamento(model.getVar(i).get(GRB.DoubleAttr.RC), 4) + ", ");
            }
            System.out.println(arrotondamento(model.getVar(numVar - 1).get(GRB.DoubleAttr.RC), 4) + "]");

        // SOLUZIONE OTTIMA MULTIPLA
            int coefficienteCostoRidotto_zero = 0;
            int varMaggioriDiZero = 0;
            boolean solMultipla;
            boolean solDegenere;
            for (GRBVar var : model.getVars()) {
                if (var.get(GRB.DoubleAttr.X) > 0)
                    varMaggioriDiZero++;

                if (Math.abs(var.get(GRB.DoubleAttr.RC)-0)<EPSILON)
                    coefficienteCostoRidotto_zero++;
            }

            if (coefficienteCostoRidotto_zero > model.getConstrs().length) //num vincoli = num variabili in base, variabili in base hanno CR = 0
                // delle variabili fuori base hanno CR = 0
                solMultipla = true;
            else
                //solo le variabili in base hanno CR = 0
                 solMultipla = false;
            System.out.print("soluzione ottima multipla: ");
            System.out.println(solMultipla ? "Si" : "No");

        // SOLUZIONE OTTIMA DEGENERE, se almeno una variabile in base ha valore  = 0
            if(varMaggioriDiZero < model.get(GRB.IntAttr.NumConstrs)) {    // variabili > 0 in base, numero variabili in base = numero di vincoli
                //ci sono variabili in base = 0, soluzione degenere
                solDegenere = true;
            } else {
                //tutte le variabili in base sono != 0, soluzione non degenere
                solDegenere = false;
            }
            System.out.print("soluzione ottima degenere: ");
            System.out.println(solDegenere ? "Si" : "No");

        //VINCOLI VERTICE OTTIMO
            System.out.println("vincoli vertice ottimo: ");
            int n = 0;
            for (int j = 0; j < yh.length; j++) {
                if (Math.abs(yh[j].get(GRB.DoubleAttr.X)-0)<EPSILON) {
                    System.out.println(model.getConstr(j).get(GRB.StringAttr.ConstrName));
                }
            }

//-------------------------------------------------------------------------------------------------------------

             System.out.println("\n");

//---------------------------- QUESITO 3 ---------------------------------------------------------------------------


        //-------------------PRIMO METODO: DUE FASI----------------------
          System.out.println("QUESITO III:");
          System.out.println("");
          boolean solAmmissibile = true;
          if(Math.abs(modelAggiuntivo.get(GRB.DoubleAttr.ObjVal)-0)<EPSILON){ //valore funzione obiettivo = 0
              for(int i=0; i<numAus;i++){
                  if(!(Math.abs(aus[i].get(GRB.DoubleAttr.X)-0)<EPSILON)){    //variabili ausiliarie = 0
                      //la soluzione del problema ausiliario è una soluzione ammissibile del problema principale
                      solAmmissibile=false;
                  }
              }
          }
          if(solAmmissibile){
              //stampa soluzione ammissibile trovata
              System.out.println("Per la soluzione ammissibile 1:");
              for(int j=0; j<xijAus.length; j++){
                  for(int k=0; k<xijAus[0].length; k++) {
                      System.out.println(xijAus[j][k].get(GRB.StringAttr.VarName) + ": " + arrotondamento(xijAus[j][k].get(GRB.DoubleAttr.X), 4));
                  }
              }
              for(int j=0; j<yhAus.length; j++){
                  System.out.println(yhAus[j].get(GRB.StringAttr.VarName) + ": " + arrotondamento(yhAus[j].get(GRB.DoubleAttr.X), 4));
              }
          }
        //------------------SECONDO METODO: MEDIA-----------------------
            // trovo soluzione ammissibile con combinazione convessa; z=aX+(1-a)Y,  a=0.5
            System.out.println(" ");
          System.out.println("Per la soluzione ammissibile 2:");
          double val;
          for(int j=0; j<xijAus.length; j++){
              for(int k=0; k<xijAus[0].length; k++) {
                  val=(xijAus[j][k].get(GRB.DoubleAttr.X)+xij[j][k].get(GRB.DoubleAttr.X))/2.0;
                  System.out.println(xij[j][k].get(GRB.StringAttr.VarName) + ": " +  arrotondamento(val, 4));
              }
          }
          for(int j=0; j<yhAus.length; j++){
              val=(yhAus[j].get(GRB.DoubleAttr.X)+yh[j].get(GRB.DoubleAttr.X))/2.0;
              System.out.println(yh[j].get(GRB.StringAttr.VarName) + ": " + arrotondamento(val, 4));
          }
        //------------------TERZO METODO: RESTRIZIONE SLACK-----------------------
          int f=0;
          while(!(yh[f].get(GRB.DoubleAttr.X)<EPSILON)){
              f++;
          }
          GRBLinExpr ex = new GRBLinExpr();
          ex.addTerm(1, yh[f]);
          model.addConstr(ex, GRB.GREATER_EQUAL, 1, "slack");
          model.optimize();
          System.out.println("");
          System.out.println("Per la soluzione ammissibile 3:");
          for(int j=0; j<xij.length; j++){
              for(int k=0; k<xijAus[0].length; k++) {
                  System.out.println(xij[j][k].get(GRB.StringAttr.VarName) + ": " +  arrotondamento(xij[j][k].get(GRB.DoubleAttr.X), 4));
              }
          }
          for(int j=0; j<yh.length; j++){
              System.out.println(yh[j].get(GRB.StringAttr.VarName) + ": " + arrotondamento(yh[j].get(GRB.DoubleAttr.X), 4));
          }
//-------------------------------------------------------------------------------------------------------------------------------------------------
        } catch (GRBException e) {
            e.printStackTrace();
        }
    }
}





