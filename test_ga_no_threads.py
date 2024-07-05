# If you on a Windows machine with any Python version 
# or an M1 mac with any Python version
# or an Intel Mac with Python > 3.7
# the multi-threaded version does not work
# so instead, you can use this version. 

import unittest
import population
import simulation 
import genome 
import creature 
import numpy as np
import csv

class TestGA(unittest.TestCase):
    def testBasicGA(self, pop_size, gene_count, point_mutate_rate, shrink_mutate_rate, grow_mutate_rate):
        pop = population.Population(pop_size=pop_size, 
                                    gene_count=gene_count)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation()

        results = []
        elites = []

        for iteration in range(40):
            # this is a non-threaded version 
            # where we just call run_creature instead
            # of eval_population
            for cr in pop.creatures:
                sim.run_creature(cr, 2400)            
            #sim.eval_population(pop, 2400)

            fits = [cr.get_distance_travelled() 
                    for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) 
                    for cr in pop.creatures]
            
            print(iteration, "fittest:", np.round(np.max(fits), 3), 
                  "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links)))       
            results.append([iteration, np.round(np.max(fits), 3), np.round(np.mean(fits), 3), np.round(np.mean(links)), np.round(np.max(links))])

            

            #GA operators
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []

            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]

                # crossover and mutations
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                dna = genome.Genome.grow_mutate(dna, rate=0.1)

                #create new creature
                cr = creature.Creature(1)
                cr.update_dna(dna)
                new_creatures.append(cr)

            # elitism
            max_fit = np.max(fits)
            
            for cr in pop.creatures:
                if cr.get_distance_travelled() == max_fit:
                    new_cr = creature.Creature(1)
                    new_cr.update_dna(cr.dna)

                    new_creatures[0] = new_cr
                    filename = "elite_"+str(iteration)+".csv"
                    genome.Genome.to_csv(cr.dna, filename)

                    break
            
            pop.creatures = new_creatures

            results.append([iteration, np.max(fits), np.mean(fits), np.mean(links), np.max(links)])

            #assert that first fitness is not 0
            self.assertNotEqual(fits[0], 0)

        with open('experiment_results.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Iteration", "Max Fitness", "Mean Fitness", "Mean Links", "Max Links"])
            writer.writerows(results)



        return results

    
    def test_multiple(self):
        experiments = [
            {"pop_size": 10, "gene_count": 3, "point_mutate_rate": 0.1, "shrink_mutate_rate": 0.25, "grow_mutate_rate": 0.1},
            {"pop_size": 20, "gene_count": 4, "point_mutate_rate": 0.2, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.2},
            {"pop_size": 50, "gene_count": 3, "point_mutate_rate": 0.2, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.2},
            {"pop_size": 50, "gene_count": 5, "point_mutate_rate": 0.3, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.2},
            # Variations in mutation rates
            {"pop_size": 20, "gene_count": 4, "point_mutate_rate": 0.3, "shrink_mutate_rate": 0.1, "grow_mutate_rate": 0.3},
            {"pop_size": 20, "gene_count": 4, "point_mutate_rate": 0.4, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.1},
            # Larger population sizes
            {"pop_size": 100, "gene_count": 5, "point_mutate_rate": 0.2, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.2},
            {"pop_size": 200, "gene_count": 6, "point_mutate_rate": 0.3, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.2},
            # More complex gene structures
            {"pop_size": 50, "gene_count": 10, "point_mutate_rate": 0.2, "shrink_mutate_rate": 0.2, "grow_mutate_rate": 0.2},
        ]

        with open('experiments_result.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Pop Size", "Gene Count", "Point Mutate Rate", "Shrink Mutate Rate", "Grow Mutate Rate", "Iteration", "Max Fitness", "Mean Fitness", "Mean Links", "Max Links"])

            for idx, params in enumerate(experiments):
                results = self.testBasicGA(
                    pop_size=params["pop_size"],
                    gene_count=params["gene_count"],
                    point_mutate_rate=params["point_mutate_rate"],
                    shrink_mutate_rate=params["shrink_mutate_rate"],
                    grow_mutate_rate=params["grow_mutate_rate"]
                )
                for result in results:
                    writer.writerow([params["pop_size"], params["gene_count"], params["point_mutate_rate"], params["shrink_mutate_rate"], params["grow_mutate_rate"]] + result)

            #cant get creature to climb mountain so cant test duration completed..... 

unittest.main()