# Aeromodelo Astro-Cargo: modelo e arquivos para impressão

> 🌐 Esta página é uma tradução do [original em russo](../../../../airframe/README.md). Se a tradução e o original divergirem, vale o original. O firmware exibe as mensagens do console em russo, por isso elas são citadas como estão.

Aqui está o próprio avião: o projeto do Fusion 360 e os arquivos STL para impressão 3D. A eletrônica e a placa da controladora de voo estão descritas em [FC_BOARD.md](../FC_BOARD.md); a montagem e o primeiro voo, no [guia do piloto](../PILOT_GUIDE.md).

## Versão do modelo: v2

Esta pasta contém o **Astro-Cargo v2**. Não haverá v1 no repositório: o primeiro modelo não é publicado, então o segundo passou a ser o primeiro a ser divulgado.

- [Projeto do Fusion 360](../../../../airframe/fusion360/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.f3d), 14 MB;
- [Arquivo STL para impressão](../../../../airframe/stl/Astro-Cargo%20v2%20%28Bad%20wheel%20%26%20no%20battery%20mount%29.stl), 6 MB.

Os arquivos têm esses nomes para que fique claro de imediato o que há de errado nesta versão (detalhes abaixo).

> [!WARNING]
> **Foram encontradas falhas críticas de projeto na v2:**
>
> 1. **A fixação do trem de pouso na fuselagem é fraca demais.** Ela não aguenta o peso do avião, e a fuselagem rasga no ponto de fixação.
> 2. **Não há suporte para a fita de velcro que prende a bateria.**
>
> As duas falhas serão corrigidas no próximo protótipo, o **Astro-Cargo v3**. O modelo dele aparecerá nesta pasta assim que ficar pronto. Até lá, não voe com a v2 sem modificações: reforce você mesmo a fixação do trem de pouso e providencie um lugar para a fita de velcro.

## O que fica onde

| Pasta | O que há nela |
|---|---|
| [`fusion360/`](../../../../airframe/fusion360/) | O projeto-fonte: um arquivo `.f3d` (ou um pacote `.f3z`, se o projeto tiver vários arquivos). Dele dá para mudar as dimensões e exportar as peças de novo |
| [`stl/`](../../../../airframe/stl/) | Peças prontas para imprimir, em formato STL |

## Como nomear os arquivos

- Os nomes usam letras latinas e levam o número da versão. Os arquivos da v2 foram nomeados de modo que o nome já fale das falhas, mas daqui em diante é melhor evitar espaços e parênteses: `astro-cargo_v3.f3d`, `astro-cargo_v3.stl`. Assim fica mais fácil apontar para o arquivo a partir da documentação. Se houver várias peças, cada uma fica em seu próprio arquivo: `fuselage_v3.stl`, `wing_left_v3.stl`.
- Os arquivos da v3 ficarão ao lado (`astro-cargo_v3.f3d`), e os da v2 permanecerão: assim dá para ver o que exatamente foi corrigido.
- As unidades são milímetros. Se o projeto usar outras, informe isso junto ao arquivo.

## Se o arquivo for grande demais

O GitHub não aceita arquivos maiores que 100 MB e já avisa a partir de 50 MB. Por isso, confira o tamanho do arquivo antes do commit. Esses arquivos não devem ir para o repositório; coloque-os:

- na seção **Releases** do GitHub: o arquivo pode ser anexado a uma versão publicada e pode ter até 2 GB;
- no [Git LFS](https://git-lfs.com), se o arquivo precisar ficar no próprio repositório e mudar junto com o código;
- em uma hospedagem externa, adicionando o link a este README.

O git trata os arquivos `.stl`, `.f3d`, `.f3z`, `.step`, `.stp` e `.3mf` desta pasta como binários (veja [`.gitattributes`](../../../../.gitattributes)): ele não altera as quebras de linha deles e não mostra diferenças linha a linha.

## Licença

O modelo é distribuído nos mesmos termos de todo o projeto: a [OpenPlane License](../LICENSE.md), ou seja, MIT com crédito obrigatório ao autor, proibição de uso militar e proibição de causar dano intencional a pessoas ou bens sem o consentimento deles. Você pode imprimir, modificar e melhorar o modelo dentro desses termos, mas precisa creditar o autor, Damir Lebedev (Damn / Проклятый).
