import typer
import inquirer
import logging
from cg.navegacao_reativa import main as main_reativa
from cg.navegacao_mapa import main as main_mapa

logging.basicConfig(level=logging.INFO)

# Inicializa a aplicação Typer para criar uma CLI (Command Line Interface)
app = typer.Typer()

# processa a resposta e diz como seguir
def processar_respostas(respostas: dict):
    opcao = respostas.get("opcao")
    if not opcao:
        logging.error("Nenhuma opção foi selecionada. Encerrando.")
        typer.echo("Nenhuma opção selecionada. Encerrando.")
        raise typer.Exit()
    
    # executa a navegação reativa
    if opcao == "Navegação Reativa":
        logging.info("Iniciando navegação reativa...")
        typer.echo("Iniciando navegação reativa...")
        main_reativa()
    
    # executa a navegação por mapa
    elif opcao == "Navegação por Mapa":
        logging.info("Iniciando navegação por mapa...")
        typer.echo("Iniciando navegação por mapa...")
        main_mapa()
    else:
        logging.error(f"Opção inválida selecionada: {opcao}")
        typer.echo(f"Opção inválida: {opcao}")
        raise typer.Exit(code=1)

@app.command()
def navegacao():
    
    # menu do usuario
    perguntas = [
        inquirer.List(
            "opcao",
            message="Escolha o tipo de navegação",
            choices=["Navegação Reativa", "Navegação por Mapa"],
        ),
    ]
    respostas = inquirer.prompt(perguntas)  
    if respostas:
        processar_respostas(respostas)
    else:
        typer.echo("Nenhuma entrada detectada. Encerrando aplicação.")
        logging.warning("Nenhuma entrada detectada. Aplicação foi encerrada.")
        raise typer.Exit()

# função que inicializa a cli
def main():
    typer.echo("Bem-vindo à interface de navegação!")
    app()

if __name__ == "_main_":
    main()
