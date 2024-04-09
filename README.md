
## FS-MAC
FS-MAC é uma arquitetura que promove a flexibilização da subcamada MAC de redes sem fio. Esta arquitetura foi desenvolvida para permitir o uso de mais de um protocolo no controle de acesso ao meio, utilizando cada um na situação em que é mais eficiente.

##Protótipo
O protótipo nesse projeto é uma implementação da arquitetura FS-MAC. Ele contém 6 blocos GNU Radio que permitem o completo funcionamento da plataforma.

## Dependências
Esse protótipo figura como a subcamada MAC da pilha ZigBee e por isso usou a implementação dessa pilha como base para a implementação. Essa e mais algumas dependências estão listadas abaixo:
- https://github.com/bastibl/gr-ieee802-15-4 (Pilha ZigBee utilizada como base)
- https://github.com/bastibl/gr-foo
- https://github.com/osh/gr-eventstream
- https://github.com/osh/gr-uhdgps

## Instalação
Após instalar as dependências, no diretório raiz **gr-fsmac** execute os seguintes comandos:
```
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
```
## Exemplos
No diretório **gr-fsmac/examples** existem três flow graphs já montados para teste. Um deles utiliza o CSMA como camada MAC, outro utiliza o TDMA e outro utiliza a composição da arquitetura FS-MAC substituindo a subcamada MAC da pilha ZigBee.
