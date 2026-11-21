from src.ML.bots import Ml

bot = Ml()
try:
    model = bot.train(100_000)
    model.save("balancing_bot")
finally:
    bot.env.close()