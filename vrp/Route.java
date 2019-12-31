/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package vrp;

import java.util.ArrayList;

/**
 *
 * @author mzaxa
 */
public class Route {
        
    ArrayList <Node> nodes = new ArrayList();
        double cost;
        double load;
        double capacity;
        
        public Route(double cap) 
        {
            cost = 0;
            nodes = new ArrayList();
            load = 0;
            capacity = cap;
        }
}
