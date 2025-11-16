# 🛰️ Trabalho Prático – SAVI
# Eduardo Pereira - 108009
# Gonçalo Almeida - 108432 
## Registo de Nuvens de Pontos RGB-D com ICP

Este projeto teve como objetivo trabalhar com registo de nuvens de pontos RGB-D usando o algoritmo ICP, tanto através das ferramentas já prontas do Open3D como através de uma implementação personalizada construída por nós. Na ultima tarefa, o objetivo foi otimizar os parametros de uma esfera de forma a que ele inclua todos os pontos de duas pointclouds, de forma a que tenha o menor raio possivel. O dataset usado foi o TUM fornecido pelo professor nas aulas praticas, contendo pares RGB + profundidade, a partir dos quais gerámos as respetivas point clouds.

---

# Pré-processamento

Antes de aplicarmos o ICP, tratámos todos os pares RGB-D da mesma forma. Começámos por carregar as imagens com o OpenCV, garantindo que a profundidade se mantinha em `uint16`. Convertêmo-las para objetos do Open3D e criámos imagens RGBD mantendo as cores reais.

A partir daí gerámos as point clouds com os os intrínsecos oficiais do TUM dataset. Como o Open3D usa um sistema de coordenadas diferente, aplicámos uma transformação para corrigir a orientação. Fizemos também voxel downsampling para reduzir a densidade e calculámos normais para as puder usar no point to plane.

---

# Tarefa 1 – ICP com Open3D

Nesta tarefa utilizámos diretamente o ICP do Open3D. Isto serviu como referência para comparar depois com a implementação personalizada. Usámos o método Point-to-Plane, que costuma ser mais estável e convergir mais rapidamente.

Bastou definir os parâmetros principais (threshold, transformação inicial e método de estimativa) e visualizar as nuvens antes e depois. No fim também convertemos a matriz 4×4 resultante para um vetor de 6 parâmetros, útil para usar essa transformação como referencia na Tarefa 2.

### Visualização  

**Antes do ICP:**  
> !(¨T1_ANTES.png")

**Depois do ICP:**  
> !("T2_DEPOIS.png")

Os resultados foram bons — as points clouds alinharam-se de forma bastante eficiente e precisa.

---

# Tarefa 2 – ICP Personalizado (SciPy)

Aqui construímos o ciclo ICP completo manualmente. Isto incluiu: encontrar correspondências, calcular a função de erro, usar o `least_squares` do SciPy para obter incrementos de transformação e aplicar esses incrementos iterativamente.

Em cada iteração:
- encontrámos os vizinhos mais próximos com KD-Tree,
- calculámos as distancias individuais ponto-a-ponto (erro),
- pedimos ao `least_squares` para minimizar esse erro,
- aplicámos a nova transformação incremental e seguimos para a iteração seguinte.
- Verificamos se o resultado já tinha convergido e nesse caso termina o loop.

O maior desafio desta tarefa foi perceber como usar corretamente o `scipy.optimize.least_squares` dentro do ciclo ICP. No início não sabíamos muito bem como montar a função de erro corretamente, nem como passar os pontos correspondentes ao otimizador. Para complicar, também não estava claro como aplicar a transformação devolvida pelo `least_squares` de forma incremental ao longo das iterações.


### 🖼️ Visualizações  


**Antes do ICP personalizado:**  
> !(T2_ANTES.png)

**Depois do ICP personalizado:**  
> !(T2_DEPOISS.png)

### 🖥️ Saída do terminal  
> !(T2_TERMINAL.png)

O resultado final ficou bastante próximo do obtido com o ICP do Open3D, o que confirmou que a nossa implementação estava correta e funcional.

---

# Comparação entre as duas abordagens

O ICP do Open3D foi claramente mais rápido, já que é bastante otimizado. A nossa versão personalizada foi muito útil para compreender em detalhe o funcionamento interno do algoritmo. Embora precise de mais afinações para ser tão estável quanto a versão do Open3D, produziu resultados bastante semelhantes.

---

# Conclusão

Este trabalho permitiu-nos entender melhor como gerar point clouds RGB-D, como trabalhar com o sistema de coordenadas do Open3D, e sobretudo como funciona o ICP tanto numa versão pronta como na construção manual.

No final ficámos com uma boa perceção da diferença entre usar uma biblioteca altamente otimizada e implementar o ciclo completo por conta própria, o que foi bastante enriquecedor para a compreensão do processo.

---
