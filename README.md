# ActuadoresSuavesDispositivosRoboticosRehabilitacion

Este trabajo se centra en el desarrollo e implementación de estrategias de control basadas en inteligencia artificial para dispositivos robóticos blandos aplicados en el ámbito de la rehabilitación. Ante la creciente demanda de soluciones que combinen seguridad, adaptabilidad y eficacia en terapias de rehabilitación, este trabajo propone un marco de control innovador que utilice técnicas de \textit{Deep Reinforcement Learning} (DRL) y que integre dichas técnicas en enfoques híbridos con el control convencional más conocido, con la intención de superar las limitaciones inherentes al modelado y control de actuadores blandos, caracterizados por su alta no linealidad y acoplamientos complejos.

Se revisa el estado del arte en actuadores suaves y dispositivos robóticos en asistencia y rehabilitación, estableciendo una base teórica sólida que abarca tanto métodos tradicionales de control como técnicas emergentes \textit{data-driven}. Se desarrolla, mediante simulaciones, un dispositivo robótico destinado a la rehabilitación de los dedos, en el cual se analizan distintas estrategias de control: (i) un agente de RL entrenado con el algoritmo TD3 y arquitectura MLP, (ii) un agente de RL basado en el algoritmo PPO y arquitectura LSTM, y (iii) un control proporcional con ajuste dinámico de la ganancia, optimizado a través de un agente de RL entrenado con el algoritmo TD3.

Los resultados obtenidos en el entorno simulado evidencian que, si bien los métodos \textit{model-free} ofrecen capacidad de adaptación ante las dinámicas inciertas de los actuadores blandos, la incorporación de estrategias híbridas puede mejorar significativamente la robustez y precisión del sistema, así como simplificar la gestión de las restricciones. Además, se destaca la importancia de un diseño cuidadoso de la función de recompensa y la configuración adecuada de los hiperparámetros en el proceso de entrenamiento del agente de RL.

También se proponen nuevas líneas de trabajo orientadas a la integración de modelos predictivos y técnicas de aprendizaje por refuerzo, con vistas a la eventual validación experimental en entornos médicos reales.
